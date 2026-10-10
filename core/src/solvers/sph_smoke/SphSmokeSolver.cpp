#include "fluids/solvers/sph_smoke/SphSmokeSolver.h"

#include <algorithm>
#include <cmath>

#include "fluids/solvers/sph/Kernels.h"

namespace fluids::sph_smoke {

using fluids::sph::poly6;
using fluids::sph::viscosityLaplacian;

namespace {

// Fixed per-particle mass used only for the internal SPH kernel-density
// estimate that normalizes the viscosity term (see computeSphDensity/
// computeForces below). Unlike sph::SPHParams::particleMass (water), this is
// NOT exposed as a tunable and NOT calibrated against any rest density: this
// solver has no EOS to calibrate for, and (see the derivation in
// computeForces' comment) the viscosity acceleration this mass ultimately
// feeds into scales as 1/kParticleMass overall — an arbitrary fixed value is
// exactly as valid as any other, because the exposed `viscosity` parameter
// already provides a free, live-tunable overall scale that fully compensates
// for whatever fixed value is chosen here. Fixed at 1.0 for simplicity, same
// value MpmSmokeSolver uses for the analogous reason.
constexpr float kParticleMass = 1.0f;

// Below this density, a particle is considered fully faded and is culled
// (see SphSmokeSolver::cullFaded). Same value and reasoning as
// MpmSmokeSolver's kDensityCullThreshold.
constexpr float kDensityCullThreshold = 0.01f;

// --- Emission tuning (same approach as MpmSmokeSolver, see its .cpp for the
// full derivation) -----------------------------------------------------
//
// SmokeScenario::emissionRate has no natural "per grid cell" meaning for a
// particle-based source, so it's translated directly into a particle spawn
// rate: particlesPerSecond = emissionRate * kParticlesPerSecondPerEmissionUnit.
// Reusing MpmSmokeSolver's exact constant (20.0) rather than deriving a new
// one keeps the two particle-based smoke solvers' particle counts/visual
// density directly comparable for a given SmokeScenario — a deliberate
// consistency choice, not an oversight; empirical testing (see
// docs/sph-smoke-sim-api.md) found this rate gives this solver's
// smoothingRadius enough local particle density for the viscosity kernel to
// have a visible smoothing effect without needing a different constant.
constexpr float kParticlesPerSecondPerEmissionUnit = 20.0f;

// Same emission-credit-banking cap and reasoning as MpmSmokeSolver.
constexpr float kMaxBankedEmission = 50.0f;

// Same small random velocity jitter (so a burst of same-tick particles
// doesn't move in lockstep before buoyancy/viscosity take over) and
// density/temperature jitter ranges as MpmSmokeSolver, for consistency.
constexpr float kInitialVelocityJitterRange = 0.1f; // m/s, full range (+/- half)
constexpr float kInitialDensityMin = 0.85f;
constexpr float kInitialTemperatureFrac = 0.85f;

// Fixed RNG seed, same reasoning as MpmSmokeSolver: deterministic emission
// jitter per reset() so a given scenario/params/step sequence is
// reproducible for tests, at no visual cost.
constexpr std::uint32_t kRngSeed = 1337u;

} // namespace

void SphSmokeSolver::reset(const SmokeScenario& scenario, const SphSmokeParams& params) {
    scenario_ = scenario;

    maxParticles_ = static_cast<std::size_t>(std::max(params.maxParticles, 1));
    positions_.assign(maxParticles_, Vec2());
    velocities_.assign(maxParticles_, Vec2());
    density_.assign(maxParticles_, 0.0f);
    temperature_.assign(maxParticles_, params.ambientTemperature);

    accelerationsScratch_.assign(maxParticles_, Vec2());
    sphDensity_.assign(maxParticles_, 0.0f);
    activePositionsScratch_.clear();
    neighborScratch_.clear();

    activeCount_ = 0;
    emitAccumulator_ = 0.0f;
    rng_.seed(kRngSeed);
}

bool SphSmokeSolver::debugSpawnParticle(const Vec2& position, const Vec2& velocity, float density, float temperature) {
    if (activeCount_ >= maxParticles_) return false;
    const std::size_t idx = activeCount_;
    positions_[idx] = position;
    velocities_[idx] = velocity;
    density_[idx] = density;
    temperature_[idx] = temperature;
    ++activeCount_;
    return true;
}

void SphSmokeSolver::emit(const SphSmokeParams& params, float dt) {
    const float sourceArea = scenario_.sourceWidth * scenario_.sourceHeight;
    if (sourceArea <= 0.0f) return;

    const float particlesPerSecond = std::max(scenario_.emissionRate, 0.0f) * kParticlesPerSecondPerEmissionUnit;
    emitAccumulator_ = std::min(emitAccumulator_ + particlesPerSecond * dt, kMaxBankedEmission);

    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    while (emitAccumulator_ >= 1.0f) {
        // At capacity: skip the rest of this tick's emission rather than
        // recycling the oldest/faintest particle — same reasoning as
        // MpmSmokeSolver::emit (avoids a scan and a visible teleport).
        if (activeCount_ >= maxParticles_) break;

        emitAccumulator_ -= 1.0f;

        const std::size_t idx = activeCount_;
        positions_[idx] = Vec2(scenario_.sourceX + unit(rng_) * scenario_.sourceWidth,
                                scenario_.sourceY + unit(rng_) * scenario_.sourceHeight);
        velocities_[idx] = Vec2((unit(rng_) - 0.5f) * kInitialVelocityJitterRange,
                                 (unit(rng_) - 0.5f) * kInitialVelocityJitterRange);
        density_[idx] = kInitialDensityMin + unit(rng_) * (1.0f - kInitialDensityMin);
        temperature_[idx] =
            params.sourceTemperature * (kInitialTemperatureFrac + unit(rng_) * (1.0f - kInitialTemperatureFrac));
        ++activeCount_;
    }
}

void SphSmokeSolver::computeSphDensity(const SphSmokeParams& params, const UniformGrid& grid) {
    const float h = params.smoothingRadius;

    for (std::size_t i = 0; i < activeCount_; ++i) {
        neighborScratch_.clear();
        grid.queryNeighbors(activePositionsScratch_[i], h, neighborScratch_);

        // The particle always finds itself in its own neighbor query
        // (distance 0 from itself), so this sum always includes at least
        // kParticleMass * poly6(0, h) > 0 — this density estimate can never
        // be exactly zero (or even close to it) regardless of how isolated
        // a particle is, unlike a naive read of "sparse smoke -> near-zero
        // density" might suggest. This is exactly what makes dividing by it
        // in computeForces() below safe with no extra floor/clamp needed
        // (see that function's comment).
        float density = 0.0f;
        for (int j : neighborScratch_) {
            const float r = (activePositionsScratch_[i] - activePositionsScratch_[static_cast<std::size_t>(j)]).length();
            density += kParticleMass * poly6(r, h);
        }
        sphDensity_[i] = density;
    }
}

void SphSmokeSolver::computeForces(const SphSmokeParams& params, const UniformGrid& grid) {
    const float h = params.smoothingRadius;

    for (std::size_t i = 0; i < activeCount_; ++i) {
        neighborScratch_.clear();
        grid.queryNeighbors(activePositionsScratch_[i], h, neighborScratch_);

        Vec2 viscosityForce{0.0f, 0.0f};
        const float rhoI = sphDensity_[i];

        for (int jIdx : neighborScratch_) {
            const auto j = static_cast<std::size_t>(jIdx);
            if (j == i) continue; // v_j - v_i is exactly zero for self anyway; skip defensively.

            const Vec2 rij = activePositionsScratch_[i] - activePositionsScratch_[j];
            const float r = rij.length();
            if (r <= 0.0f || r > h) continue;

            const float rhoJ = sphDensity_[j];
            // rhoJ is guaranteed > 0 by the self-inclusion argument in
            // computeSphDensity's comment (every active particle contributes
            // at least its own poly6(0,h) self-term) — this check is
            // defensive only, not something normal operation should ever
            // trip.
            if (rhoJ <= 0.0f) continue;

            // Standard Laplacian viscosity (Müller et al. 2003), same
            // kernel and formula shape as sph::SPHSolver::computeForces'
            // viscosity term, just with no pressure term alongside it. This
            // diffuses relative velocity toward neighbors — the sole source
            // of inter-particle coherence in this solver (see class
            // comment).
            const float lap = viscosityLaplacian(r, h);
            viscosityForce += (velocities_[j] - velocities_[i]) * (kParticleMass * lap / rhoJ);
        }
        viscosityForce *= params.viscosity;

        // Buoyancy: a direct per-particle body force (not kernel-mediated,
        // not divided by density) — same formula/shape as the other two
        // smoke variants. This is smoke's sole driving force; there is no
        // gravity term.
        Vec2 accel{0.0f, params.buoyancy * (temperature_[i] - params.ambientTemperature)};
        if (rhoI > 0.0f) {
            accel += viscosityForce * (1.0f / rhoI);
        }
        accelerationsScratch_[i] = accel;
    }
}

void SphSmokeSolver::integrate(float dt) {
    // Symplectic (semi-implicit) Euler, same integration scheme as
    // sph::SPHSolver.
    for (std::size_t i = 0; i < activeCount_; ++i) {
        velocities_[i] += accelerationsScratch_[i] * dt;
        positions_[i] += velocities_[i] * dt;
    }
}

void SphSmokeSolver::applyDissipation(const SphSmokeParams& params, float dt) {
    const float densityDecay = std::clamp(params.densityDissipation * dt, 0.0f, 1.0f);
    const float tempRelax = std::clamp(params.temperatureDissipation * dt, 0.0f, 1.0f);

    for (std::size_t i = 0; i < activeCount_; ++i) {
        density_[i] = std::max(0.0f, density_[i] * (1.0f - densityDecay));
        temperature_[i] += (params.ambientTemperature - temperature_[i]) * tempRelax;
    }
}

void SphSmokeSolver::cullFaded() {
    std::size_t i = 0;
    while (i < activeCount_) {
        if (density_[i] < kDensityCullThreshold) {
            const std::size_t last = activeCount_ - 1;
            positions_[i] = positions_[last];
            velocities_[i] = velocities_[last];
            density_[i] = density_[last];
            temperature_[i] = temperature_[last];
            --activeCount_;
            // Don't advance i — re-check the swapped-in element.
        } else {
            ++i;
        }
    }
}

void SphSmokeSolver::clampToDomain() {
    // Particle-level-only boundary handling, same as sph::SPHSolver (which
    // also has no grid to layer a grid-level boundary step on top of — this
    // solver's UniformGrid is a neighbor-search structure only, not a
    // velocity-carrying MAC/node grid the way FLIP/MPM's are). Unlike water
    // SPH's damped-bounce, this zeroes the outward velocity component
    // instead of reflecting it — same "just stop at the wall" convention
    // the other two smoke variants use (a gas venting into a wall
    // shouldn't bounce back elastically).
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

void SphSmokeSolver::step(const SphSmokeParams& params, float dt) {
    // Same defensive dt clamp as every other solver's step().
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    emit(params, dt);

    // Copy the active prefix into a scratch buffer so the UniformGrid never
    // sees stale/inactive entries beyond activeCount_ (the fixed-capacity
    // buffers hold maxParticles_ slots, but only the first activeCount_ are
    // live) — indices into this scratch vector line up 1:1 with the active
    // particle indices, so neighbor query results can index directly into
    // positions_/velocities_/etc. without any translation.
    activePositionsScratch_.assign(positions_.begin(), positions_.begin() + static_cast<std::ptrdiff_t>(activeCount_));

    if (activeCount_ > 0) {
        UniformGrid grid(std::max(params.smoothingRadius, 1e-4f));
        grid.build(activePositionsScratch_);

        computeSphDensity(params, grid);
        computeForces(params, grid);
        integrate(dt);
    }

    applyDissipation(params, dt);
    cullFaded();
    clampToDomain();
}

} // namespace fluids::sph_smoke
