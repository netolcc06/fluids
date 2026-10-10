#include "fluids/solvers/mpm_smoke/MpmSmokeSolver.h"

#include <algorithm>
#include <cmath>

namespace fluids::mpm_smoke {

using mpm::MpmGrid;

namespace {

// Same low-side index padding as mpm::MpmSolver, for the same reason (a
// particle sitting exactly at x=0/y=0 produces a logical stencil base of -1
// — see mpm::MpmSolver.cpp's kGridPadding comment for the full derivation).
// Duplicated rather than shared: this is a small, self-contained constant
// tightly coupled to the stencil math below, matching this project's
// established precedent of not force-sharing small solver-internal helpers
// across independent solvers (see FlipSmokeSolver.cpp's comment on
// duplicating bilinearCoeffs rather than sharing it with FlipSolver).
constexpr int kGridPadding = 2;

// Grid-velocity boundary margin, in cells — identical role and value to
// mpm::MpmSolver's kBoundaryMarginCells: nodes within this many cells of a
// domain wall have their outward-pointing velocity component zeroed.
constexpr int kBoundaryMarginCells = 3;

constexpr float kMinNodeMass = 1e-9f;

// Fixed per-particle mass for the APIC P2G/G2P transfer. Unlike
// mpm::MpmSolver, this is NOT calibrated from a rest density: with no
// stress/pressure force reading it, mass only ever appears as a shared
// positive weight in a momentum-weighted average (P2G scatters mass*v,
// updateGrid divides accumulated momentum by accumulated mass) — since
// every particle uses the same value, it cancels out of that division
// exactly, so its absolute magnitude is arbitrary. Fixed at 1.0 rather than
// exposed as a parameter (see MpmSmokeParams' class comment).
constexpr float kParticleMass = 1.0f;

// Below this density, a particle is considered fully faded and is culled
// (see MpmSmokeSolver::cullFaded). Matches the task's suggested threshold —
// small enough that culling never visibly clips a still-recognizable puff,
// large enough that particles don't linger indefinitely at a
// density so small it can never usefully render.
constexpr float kDensityCullThreshold = 0.01f;

// --- Emission tuning -------------------------------------------------
//
// SmokeScenario::emissionRate is defined (see FlipSmokeSolver/SmokeScenario)
// as "density units added per second within the source region" for the grid
// solver — a resolution-dependent quantity there (it's added per CELL per
// second, so the total density content added per second scales with how
// many cells the source rectangle covers). There's no equivalent notion of
// "cells" for a particle-based source, so this solver instead translates
// emissionRate directly into a particle spawn rate:
//
//   particlesPerSecond = emissionRate * kParticlesPerSecondPerEmissionUnit
//
// kParticlesPerSecondPerEmissionUnit was chosen empirically so that
// makePlumeScenario()'s default emissionRate=2.0 spawns a visually
// recognizable, continuously-renewed stream of particles (~40/s) while
// keeping the steady-state population (particlesPerSecond * average
// lifetime to the culling threshold, see below) comfortably under the
// default maxParticles=4000 — see docs/mpm-smoke-sim-api.md for the
// measured steady-state count. This does not scale with the source
// rectangle's area the way the grid version's total flux implicitly does;
// a much larger/smaller custom source rectangle changes the emitted
// particles' spatial spread but not their count-per-second, a deliberate
// simplification documented as a known difference from the grid version.
constexpr float kParticlesPerSecondPerEmissionUnit = 20.0f;

// Cap on how much "emission credit" can bank up while emission is being
// skipped at capacity (see emit()) — without this, a very long stretch at
// capacity followed by a mass-cull (e.g. a sudden parameter change) could
// otherwise release a large one-tick burst of new particles instead of a
// smooth resumption.
constexpr float kMaxBankedEmission = 50.0f;

// Newly-spawned particles get a small symmetric random velocity jitter on
// both axes (not purely vertical) so a burst of particles doesn't all move
// in lockstep before buoyancy and the APIC transfer take over — small
// enough to be visually negligible against buoyancy's own acceleration.
constexpr float kInitialVelocityJitterRange = 0.1f; // m/s, full range (+/- half)

// Newly-spawned particles get density in [kInitialDensityMin, 1.0] and
// temperature in [kInitialTemperatureFrac * sourceTemperature,
// sourceTemperature] — a little variance for visual texture in a cluster of
// particles spawned the same tick, per the task's "small jittered cluster"
// framing, rather than every new particle being bit-for-bit identical.
constexpr float kInitialDensityMin = 0.85f;
constexpr float kInitialTemperatureFrac = 0.85f;

// Fixed RNG seed: emission jitter is deterministic per reset() rather than
// wall-clock/hardware seeded, so native tests (and any caller) get
// reproducible particle streams for a given scenario/params/step sequence —
// no other solver in this project uses randomness yet, so there's no
// existing convention to match; determinism was chosen because it makes
// test failures reproducible and costs nothing visually (still looks like a
// random jittered stream, just the same "random" stream every run).
constexpr std::uint32_t kRngSeed = 1337u;

// Quadratic B-spline weights and stencil helper, identical math to
// mpm::MpmSolver.cpp's (same MLS-MPM stencil), duplicated here rather than
// shared for the same reason kGridPadding is duplicated above.
void quadraticWeights(float f, float w[3]) {
    const float f0 = 1.5f - f;
    const float f1 = f - 1.0f;
    const float f2 = f - 0.5f;
    w[0] = 0.5f * f0 * f0;
    w[1] = 0.75f - f1 * f1;
    w[2] = 0.5f * f2 * f2;
}

struct Stencil {
    int baseX = 0, baseY = 0;
    float fx = 0.0f, fy = 0.0f;
    float wx[3] = {0, 0, 0};
    float wy[3] = {0, 0, 0};
};

Stencil computeStencil(const Vec2& p, float dx, int padding) {
    Stencil s;
    const float xOverDx = p.x / dx;
    const float yOverDx = p.y / dx;
    const int logicalBaseX = static_cast<int>(std::floor(xOverDx - 0.5f));
    const int logicalBaseY = static_cast<int>(std::floor(yOverDx - 0.5f));
    s.fx = xOverDx - static_cast<float>(logicalBaseX);
    s.fy = yOverDx - static_cast<float>(logicalBaseY);
    quadraticWeights(s.fx, s.wx);
    quadraticWeights(s.fy, s.wy);
    s.baseX = logicalBaseX + padding;
    s.baseY = logicalBaseY + padding;
    return s;
}

} // namespace

void MpmSmokeSolver::reset(const SmokeScenario& scenario, const MpmSmokeParams& params) {
    scenario_ = scenario;

    maxParticles_ = static_cast<std::size_t>(std::max(params.maxParticles, 1));
    positions_.assign(maxParticles_, Vec2());
    velocities_.assign(maxParticles_, Vec2());
    C_.assign(maxParticles_, Mat2());
    density_.assign(maxParticles_, 0.0f);
    temperature_.assign(maxParticles_, params.ambientTemperature);
    activeCount_ = 0;
    emitAccumulator_ = 0.0f;
    rng_.seed(kRngSeed);

    const float cellSize = std::max(params.cellSize, 1e-4f);
    // Same ceil + generous padding reasoning as mpm::MpmSolver::reset() —
    // the grid must cover the full domain, plus kGridPadding for the
    // low-side stencil offset and further margin so the high-side 3-node
    // stencil never reads out of bounds at the domain's far edge.
    int nx = static_cast<int>(std::ceil(scenario.domainWidth / cellSize)) + 2 * kGridPadding + 4;
    int ny = static_cast<int>(std::ceil(scenario.domainHeight / cellSize)) + 2 * kGridPadding + 4;
    nx = std::max(nx, 8);
    ny = std::max(ny, 8);
    grid_.resize(nx, ny, cellSize);
}

bool MpmSmokeSolver::debugSpawnParticle(const Vec2& position, const Vec2& velocity, float density, float temperature) {
    if (activeCount_ >= maxParticles_) return false;
    const std::size_t idx = activeCount_;
    positions_[idx] = position;
    velocities_[idx] = velocity;
    C_[idx] = Mat2();
    density_[idx] = density;
    temperature_[idx] = temperature;
    ++activeCount_;
    return true;
}

void MpmSmokeSolver::emit(const MpmSmokeParams& params, float dt) {
    const float sourceArea = scenario_.sourceWidth * scenario_.sourceHeight;
    if (sourceArea <= 0.0f) return;

    const float particlesPerSecond = std::max(scenario_.emissionRate, 0.0f) * kParticlesPerSecondPerEmissionUnit;
    emitAccumulator_ = std::min(emitAccumulator_ + particlesPerSecond * dt, kMaxBankedEmission);

    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    while (emitAccumulator_ >= 1.0f) {
        // At capacity: skip the rest of this tick's emission rather than
        // recycling the oldest/faintest particle in place. Recycling would
        // need an extra scan (or a priority structure) every step just to
        // find a candidate, and would visually teleport an existing (if
        // faint) particle to the source rather than simply pausing new
        // growth — skipping is cheaper and less visually surprising.
        // emitAccumulator_ is left as-is (not decremented) so the banked
        // credit is spent as soon as a slot frees up via culling, rather
        // than being lost; kMaxBankedEmission above bounds how large that
        // eventual burst can be.
        if (activeCount_ >= maxParticles_) break;

        emitAccumulator_ -= 1.0f;

        const std::size_t idx = activeCount_;
        positions_[idx] = Vec2(scenario_.sourceX + unit(rng_) * scenario_.sourceWidth,
                                scenario_.sourceY + unit(rng_) * scenario_.sourceHeight);
        velocities_[idx] = Vec2((unit(rng_) - 0.5f) * kInitialVelocityJitterRange,
                                 (unit(rng_) - 0.5f) * kInitialVelocityJitterRange);
        C_[idx] = Mat2();
        density_[idx] = kInitialDensityMin + unit(rng_) * (1.0f - kInitialDensityMin);
        temperature_[idx] =
            params.sourceTemperature * (kInitialTemperatureFrac + unit(rng_) * (1.0f - kInitialTemperatureFrac));
        ++activeCount_;
    }
}

void MpmSmokeSolver::particleToGrid() {
    const float dx = grid_.dx;

    for (std::size_t p = 0; p < activeCount_; ++p) {
        const Vec2& pos = positions_[p];
        const Vec2& vel = velocities_[p];
        const Mat2& C = C_[p];

        const Stencil st = computeStencil(pos, dx, kGridPadding);

        // No stress/pressure term — see class comment. The affine term is
        // purely the APIC contribution (mass * C), unlike
        // mpm::MpmSolver::particleToGrid's `C*mass - dt*4/dx^2*Vp*stress`.
        const Mat2 affine = C * kParticleMass;

        for (int oy = 0; oy < 3; ++oy) {
            for (int ox = 0; ox < 3; ++ox) {
                const int ni = st.baseX + ox;
                const int nj = st.baseY + oy;
                if (!grid_.inBounds(ni, nj)) continue; // defensive; reset() sizing keeps this unreachable
                const float weight = st.wx[ox] * st.wy[oy];
                if (weight == 0.0f) continue;

                const Vec2 dpos((static_cast<float>(ox) - st.fx) * dx, (static_cast<float>(oy) - st.fy) * dx);
                const Vec2 contribution = vel * kParticleMass + affine * dpos;

                const std::size_t idx = grid_.index(ni, nj);
                grid_.vx[idx] += weight * contribution.x;
                grid_.vy[idx] += weight * contribution.y;
                grid_.mass[idx] += weight * kParticleMass;
            }
        }
    }
}

void MpmSmokeSolver::updateGrid(const MpmSmokeParams& /*params*/) {
    const float dx = grid_.dx;
    const float margin = static_cast<float>(kBoundaryMarginCells) * dx;
    const float domainWidth = scenario_.domainWidth;
    const float domainHeight = scenario_.domainHeight;

    for (int j = 0; j < grid_.ny; ++j) {
        for (int i = 0; i < grid_.nx; ++i) {
            const std::size_t idx = grid_.index(i, j);
            const float m = grid_.mass[idx];
            if (m <= kMinNodeMass) {
                grid_.vx[idx] = 0.0f;
                grid_.vy[idx] = 0.0f;
                continue;
            }

            float vx = grid_.vx[idx] / m;
            float vy = grid_.vy[idx] / m;
            // Deliberately NO gravity here — buoyancy (applied per-particle
            // in applyBuoyancyAndDamping, after G2P) is smoke's sole driving
            // force, same contract as FlipSmokeParams (no gravity field at
            // all for smoke anywhere in this project).

            const float worldX = static_cast<float>(i - kGridPadding) * dx;
            const float worldY = static_cast<float>(j - kGridPadding) * dx;

            // Same margin-based boundary clamp as mpm::MpmSolver::updateGrid.
            if (worldX < margin && vx < 0.0f) vx = 0.0f;
            if (worldX > domainWidth - margin && vx > 0.0f) vx = 0.0f;
            if (worldY < margin && vy < 0.0f) vy = 0.0f;
            if (worldY > domainHeight - margin && vy > 0.0f) vy = 0.0f;

            grid_.vx[idx] = vx;
            grid_.vy[idx] = vy;
        }
    }
}

void MpmSmokeSolver::gridToParticles() {
    const float dx = grid_.dx;
    const float invDx2 = 1.0f / (dx * dx);

    for (std::size_t p = 0; p < activeCount_; ++p) {
        const Vec2& pos = positions_[p];
        const Stencil st = computeStencil(pos, dx, kGridPadding);

        Vec2 newV(0.0f, 0.0f);
        Mat2 newC;

        for (int oy = 0; oy < 3; ++oy) {
            for (int ox = 0; ox < 3; ++ox) {
                const int ni = st.baseX + ox;
                const int nj = st.baseY + oy;
                if (!grid_.inBounds(ni, nj)) continue;
                const float weight = st.wx[ox] * st.wy[oy];
                if (weight == 0.0f) continue;

                const std::size_t idx = grid_.index(ni, nj);
                const Vec2 nodeVel(grid_.vx[idx], grid_.vy[idx]);
                newV += nodeVel * weight;

                const Vec2 dpos((static_cast<float>(ox) - st.fx) * dx, (static_cast<float>(oy) - st.fy) * dx);
                newC = newC + outer(nodeVel, dpos) * weight;
            }
        }
        newC = newC * (4.0f * invDx2);

        velocities_[p] = newV;
        C_[p] = newC;
        // No J to update — this solver tracks density/temperature instead
        // (see class comment), updated separately in applyDissipation().
    }
}

void MpmSmokeSolver::applyBuoyancyAndDamping(const MpmSmokeParams& params, float dt) {
    const float dampingFactor = std::clamp(1.0f - params.damping * dt, 0.0f, 1.0f);
    for (std::size_t p = 0; p < activeCount_; ++p) {
        velocities_[p].y += dt * params.buoyancy * (temperature_[p] - params.ambientTemperature);
        if (params.damping > 0.0f) {
            velocities_[p] *= dampingFactor;
        }
    }
}

void MpmSmokeSolver::advect(float dt) {
    for (std::size_t p = 0; p < activeCount_; ++p) {
        positions_[p] += velocities_[p] * dt;
    }
}

void MpmSmokeSolver::applyDissipation(const MpmSmokeParams& params, float dt) {
    const float densityDecay = std::clamp(params.densityDissipation * dt, 0.0f, 1.0f);
    const float tempRelax = std::clamp(params.temperatureDissipation * dt, 0.0f, 1.0f);

    for (std::size_t p = 0; p < activeCount_; ++p) {
        density_[p] = std::max(0.0f, density_[p] * (1.0f - densityDecay));
        temperature_[p] += (params.ambientTemperature - temperature_[p]) * tempRelax;
    }
}

void MpmSmokeSolver::cullFaded() {
    std::size_t i = 0;
    while (i < activeCount_) {
        if (density_[i] < kDensityCullThreshold) {
            const std::size_t last = activeCount_ - 1;
            positions_[i] = positions_[last];
            velocities_[i] = velocities_[last];
            C_[i] = C_[last];
            density_[i] = density_[last];
            temperature_[i] = temperature_[last];
            --activeCount_;
            // Don't advance i — re-check the swapped-in element.
        } else {
            ++i;
        }
    }
}

void MpmSmokeSolver::clampToDomain() {
    const float w = scenario_.domainWidth;
    const float h = scenario_.domainHeight;
    for (std::size_t i = 0; i < activeCount_; ++i) {
        Vec2& p = positions_[i];
        Vec2& v = velocities_[i];

        if (p.x < 0.0f) {
            p.x = 0.0f;
            if (v.x < 0.0f) v.x = 0.0f;
        } else if (p.x > w) {
            p.x = w;
            if (v.x > 0.0f) v.x = 0.0f;
        }
        if (p.y < 0.0f) {
            p.y = 0.0f;
            if (v.y < 0.0f) v.y = 0.0f;
        } else if (p.y > h) {
            p.y = h;
            if (v.y > 0.0f) v.y = 0.0f;
        }
    }
}

void MpmSmokeSolver::step(const MpmSmokeParams& params, float dt) {
    // Same defensive dt clamp as every other solver's step().
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    // See the class comment / docs/mpm-smoke-sim-api.md: empirically, a
    // single fixed-dt pass per step() call (no internal CFL-adaptive
    // subdivision) was stable across the caller dt range tested, so unlike
    // mpm::MpmSolver::step() there is no substepping loop here.
    emit(params, dt);

    grid_.clear();
    particleToGrid();
    updateGrid(params);
    gridToParticles();

    applyBuoyancyAndDamping(params, dt);
    advect(dt);

    applyDissipation(params, dt);
    cullFaded();
    clampToDomain();
}

} // namespace fluids::mpm_smoke
