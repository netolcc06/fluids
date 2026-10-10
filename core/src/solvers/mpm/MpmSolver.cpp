#include "fluids/solvers/mpm/MpmSolver.h"

#include <algorithm>
#include <cmath>

namespace fluids::mpm {

namespace {

// Extra low-side index padding on the node grid. A particle sitting exactly
// at the domain's lower/left edge (x=0 or y=0) produces a *logical* stencil
// base of -1 (floor(0/dx - 0.5) == -1, see computeStencil), which would be
// an out-of-range negative array index. Every physical grid index used to
// store/access grid_.mass/vx/vy is therefore `logicalIndex + kGridPadding`,
// so logical index -1 lands at physical index (kGridPadding - 1) >= 0 as
// long as kGridPadding >= 1. World position of physical node i is
// `(i - kGridPadding) * dx`.
constexpr int kGridPadding = 2;

// Weakly-compressible EOS exponent, fixed internally (matches SPH's Tait
// gamma=7 default) — no ask surfaced to tune this from the frontend.
constexpr float kGamma = 7.0f;

// J = det(deformation gradient) clamp range. J=1 is rest volume; J<1 is
// compressed, J>1 is expanded. These bounds are a defensive backstop against
// a particle whose local density estimate degenerates (e.g. isolated
// particle briefly flying through near-empty grid nodes) — they are wide
// enough to never engage during normal settled/at-rest simulation (see
// tests/core/test_mpm_solver.cpp's J-bounds test, which observes J staying
// within a couple percent of 1.0 for a settled column) but stop a runaway
// particle from reaching a degenerate zero or unbounded volume that would
// otherwise make the EOS pressure() blow up or vanish.
constexpr float kJMin = 0.1f;
constexpr float kJMax = 10.0f;

// Grid-velocity boundary margin, in cells: nodes within this many cells of
// a domain wall have their outward-pointing velocity component zeroed
// during updateGrid(). This is MPM's equivalent of FLIP's Solid boundary
// ring, but simpler — MPM's EOS stress already resists local compression,
// so a pressure-solve-style boundary condition isn't needed, just enough of
// a velocity clamp near the walls to keep material from crossing them.
constexpr int kBoundaryMarginCells = 3;

constexpr float kMinNodeMass = 1e-9f;

// CFL fraction for the internal adaptive sub-dt (see MpmSolver::step): the
// largest fraction of one cell width a particle is allowed to be swept
// across (relative to another particle it might collide with, e.g. one just
// stopped at the floor) within a single sub-step. This isn't a plain
// "particle doesn't cross more than a cell" advection bound (that alone
// tends to be fine up to ~1.0) — it's tighter because the *velocity
// gradient* across the 3-node stencil feeds directly into the APIC affine
// matrix C, and G2P updates J by `J *= 1 + dt*trace(C)`. trace(C) scales
// like (relativeVelocity / cellSize), so `dt * maxSpeed / cellSize` is also
// (to a rough approximation) the largest single-step fractional change in J
// the scheme can produce. Bounding that fraction to a small value keeps
// each step's J excursion — and therefore the EOS's `(1/J)^gamma` pressure
// response — from spiking hard enough to blow up before the *next*
// sub-step's smaller dt (recomputed from the new, larger velocity) can
// react. Verified empirically (see tests/core/test_mpm_solver.cpp's
// [mpm][blockdrop] test and docs/mpm-sim-api.md): a fixed dt=1/480 driving
// Block Drop's ~4.9 m/s single-mass floor impact hits this exact failure
// mode (J clamps to kJMin within one step of impact, velocity reaches
// hundreds of m/s within a handful of steps after); this CFL fraction keeps
// it stable while leaving slow-moving/settled regimes (Dam Break,
// Two-Column Collision, a resting layer) running at effectively the full
// requested dt in one sub-step, since maxSpeed there is small.
constexpr float kCflFraction = 0.1f;

// Hard cap on sub-steps per step() call — a safety net against a pathological
// infinite loop if velocities are diverging for some unrelated reason (a
// future bug, or a caller-supplied dt/params combination nothing above
// anticipated), not a value expected to be hit in normal operation. At
// kCflFraction=0.1 and the default cellSize=0.02, even a 1000 m/s particle
// only needs dt/((0.1*0.02/1000)) ~= a few hundred sub-steps to cover a
// 1/30s caller dt — comfortably under this cap for any speed this solver
// should ever plausibly reach; if the cap is hit, the remaining time is
// advanced in one final (unsafe) sub-step so step() always fully advances by
// the requested dt rather than silently falling behind.
constexpr int kMaxSubsteps = 512;

// Quadratic B-spline weights for a fractional offset f (expected in
// [0.5, 1.5) — the standard MLS-MPM stencil range, see computeStencil).
void quadraticWeights(float f, float w[3]) {
    const float f0 = 1.5f - f;
    const float f1 = f - 1.0f;
    const float f2 = f - 0.5f;
    w[0] = 0.5f * f0 * f0;
    w[1] = 0.75f - f1 * f1;
    w[2] = 0.5f * f2 * f2;
}

struct Stencil {
    int baseX = 0, baseY = 0; // physical grid index of the (offset 0,0) stencil node
    float fx = 0.0f, fy = 0.0f;
    float wx[3] = {0, 0, 0};
    float wy[3] = {0, 0, 0};
};

// base = floor(p/dx - 0.5) (logical, per axis); fx = p/dx - base. This is
// the standard MLS-MPM stencil offset so the particle sits centered over a
// 3x3 node neighborhood. Returns physical (padded) base indices, ready to
// index directly into MpmGrid.
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

void MpmSolver::seedRegion(float x, float y, float width, float height, float vx, float vy) {
    const float spacing = scenario_.spacing;
    if (spacing <= 0.0f || width <= 0.0f || height <= 0.0f) return;

    const int nx = std::max(1, static_cast<int>(width / spacing));
    const int ny = std::max(1, static_cast<int>(height / spacing));

    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            const float px = x + (static_cast<float>(ix) + 0.5f) * spacing;
            const float py = y + (static_cast<float>(iy) + 0.5f) * spacing;
            positions_.emplace_back(px, py);
            velocities_.emplace_back(vx, vy);
            C_.emplace_back(); // zero-initialized affine matrix
            J_.push_back(1.0f); // rest volume
        }
    }
}

void MpmSolver::reset(const Scenario& scenario, const MpmParams& params) {
    scenario_ = scenario;

    positions_.clear();
    velocities_.clear();
    C_.clear();
    J_.clear();

    seedRegion(scenario.regionX, scenario.regionY, scenario.regionWidth, scenario.regionHeight,
               scenario.velocityX, scenario.velocityY);
    if (scenario.hasRegion2) {
        seedRegion(scenario.region2X, scenario.region2Y, scenario.region2Width, scenario.region2Height,
                   scenario.region2VelocityX, scenario.region2VelocityY);
    }

    // particleMass ~= restDensity * spacing^2 — same calibration rule as
    // SPHParams::particleMass (mass of one rest-packed grid cell's worth of
    // area). restVolume is the corresponding rest-state per-particle area;
    // current volume at any later time is J * restVolume. See
    // docs/mpm-sim-api.md for what breaks if this falls out of sync with
    // scenario.spacing.
    particleMass_ = params.restDensity * scenario.spacing * scenario.spacing;
    restVolume_ = scenario.spacing * scenario.spacing;

    const float cellSize = std::max(params.cellSize, 1e-4f);
    // ceil (not round) so the grid always covers at least the full nominal
    // domain, plus generous padding on both sides: kGridPadding to absorb
    // the low-side stencil offset (see kGridPadding above) and a further
    // margin so the high-side 3-node stencil never reads out of bounds for
    // a particle sitting exactly at the domain's far edge (worked out in
    // the kGridPadding comment: physical index needed can reach
    // ~domainWidth/cellSize + kGridPadding + 3.5).
    int nx = static_cast<int>(std::ceil(scenario.domainWidth / cellSize)) + 2 * kGridPadding + 4;
    int ny = static_cast<int>(std::ceil(scenario.domainHeight / cellSize)) + 2 * kGridPadding + 4;
    nx = std::max(nx, 8);
    ny = std::max(ny, 8);
    grid_.resize(nx, ny, cellSize);
}

void MpmSolver::particleToGrid(const MpmParams& params, float dt) {
    const float dx = grid_.dx;
    const float invDx2 = 1.0f / (dx * dx);

    for (std::size_t p = 0; p < positions_.size(); ++p) {
        const Vec2& pos = positions_[p];
        const Vec2& vel = velocities_[p];
        const Mat2& C = C_[p];
        const float J = J_[p];

        const Stencil st = computeStencil(pos, dx, kGridPadding);

        // Weakly-compressible EOS: pressure(J) = stiffness * ((1/J)^gamma - 1).
        // 1/J == density/restDensity for a particle whose deformation
        // gradient determinant is J (J = currentVolume/restVolume, and
        // density = mass/currentVolume = restDensity/J). Clamped to >= 0,
        // same reasoning as SPH's Tait clamp: this is a repulsive-only
        // pressure response (resists compression) with no attractive force
        // when locally under-dense (J > 1), which would be unphysical for a
        // free-surface fluid (particles at/near the surface would otherwise
        // get pulled back together instead of splitting apart naturally).
        const float invJ = 1.0f / std::max(J, 1e-4f);
        float pressure = params.stiffness * (std::pow(invJ, kGamma) - 1.0f);
        pressure = std::max(pressure, 0.0f);

        Mat2 stress(-pressure, 0.0f, 0.0f, -pressure);
        if (params.viscosity > 0.0f) {
            // Cheap MLS-MPM viscosity shortcut: C already approximates the
            // local affine velocity field, so its symmetric part is a cheap
            // stand-in for the velocity gradient / strain rate. Disabled by
            // default (params.viscosity == 0); see MpmParams::viscosity.
            const Mat2 Ct = C.transpose();
            stress = stress + (C + Ct) * params.viscosity;
        }

        const float particleVolume = J * restVolume_;
        // affine = mass*C - dt * (4/dx^2) * Vp * stress
        const Mat2 affine = C * particleMass_ - stress * (dt * 4.0f * invDx2 * particleVolume);

        for (int oy = 0; oy < 3; ++oy) {
            for (int ox = 0; ox < 3; ++ox) {
                const int ni = st.baseX + ox;
                const int nj = st.baseY + oy;
                if (!grid_.inBounds(ni, nj)) continue; // defensive; reset() sizing keeps this unreachable
                const float weight = st.wx[ox] * st.wy[oy];
                if (weight == 0.0f) continue;

                const Vec2 dpos((static_cast<float>(ox) - st.fx) * dx, (static_cast<float>(oy) - st.fy) * dx);
                const Vec2 contribution = vel * particleMass_ + affine * dpos;

                const std::size_t idx = grid_.index(ni, nj);
                grid_.vx[idx] += weight * contribution.x;
                grid_.vy[idx] += weight * contribution.y;
                grid_.mass[idx] += weight * particleMass_;
            }
        }
    }
}

void MpmSolver::updateGrid(const MpmParams& params, float dt) {
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
            vy += dt * params.gravity;

            const float worldX = static_cast<float>(i - kGridPadding) * dx;
            const float worldY = static_cast<float>(j - kGridPadding) * dx;

            // Margin-based boundary clamp: zero the outward-pointing
            // component near each wall. Simpler than FLIP's full
            // Fluid/Air/Solid classification + pressure-solve boundary
            // condition — sufficient here because the EOS stress already
            // handles compressibility locally, so this only needs to keep
            // material from crossing the domain edge, not conserve
            // incompressibility globally.
            if (worldX < margin && vx < 0.0f) vx = 0.0f;
            if (worldX > domainWidth - margin && vx > 0.0f) vx = 0.0f;
            if (worldY < margin && vy < 0.0f) vy = 0.0f;
            if (worldY > domainHeight - margin && vy > 0.0f) vy = 0.0f;

            grid_.vx[idx] = vx;
            grid_.vy[idx] = vy;
        }
    }
}

void MpmSolver::gridToParticles(float dt) {
    const float dx = grid_.dx;
    const float invDx2 = 1.0f / (dx * dx);

    for (std::size_t p = 0; p < positions_.size(); ++p) {
        Vec2& pos = positions_[p];
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

        float J = J_[p] * (1.0f + dt * newC.trace());
        J_[p] = std::clamp(J, kJMin, kJMax);

        pos += newV * dt;
    }
}

void MpmSolver::clampToDomain() {
    const float w = scenario_.domainWidth;
    const float h = scenario_.domainHeight;
    for (std::size_t i = 0; i < positions_.size(); ++i) {
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

float MpmSolver::maxParticleSpeed() const {
    float maxSpeedSq = 0.0f;
    for (const Vec2& v : velocities_) {
        const float speedSq = v.x * v.x + v.y * v.y;
        maxSpeedSq = std::max(maxSpeedSq, speedSq);
    }
    return std::sqrt(maxSpeedSq);
}

void MpmSolver::substep(const MpmParams& params, float dt) {
    grid_.clear();
    particleToGrid(params, dt);
    updateGrid(params, dt);
    gridToParticles(dt);
    clampToDomain();
}

void MpmSolver::step(const MpmParams& params, float dt) {
    if (positions_.empty()) return;

    // Same defensive dt clamp as SPHSolver/FlipSolver: guards against a
    // runaway caller dt. This is NOT a substitute for driving the solver
    // with a small enough substep for EOS stability — see below and
    // docs/mpm-sim-api.md.
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    // Adaptive CFL sub-stepping: a fixed substep tuned against a gradual
    // scenario (Dam Break's progressive collapse) isn't safe for a sudden
    // high-velocity single impact (Block Drop's block hits the floor as one
    // coherent mass at ~4.9 m/s) — see kCflFraction's comment above for the
    // full mechanism. Re-derive the safe sub-dt from the *current* max
    // particle speed before every sub-step, so the loop automatically takes
    // many small sub-steps through a violent impact and one big sub-step
    // (covering the whole requested dt) the rest of the time, without the
    // caller (MpmSimulation::step / the JS contract) needing to know or care.
    const float dx = grid_.dx;
    float remaining = dt;
    int iterations = 0;
    while (remaining > 1e-8f && iterations < kMaxSubsteps) {
        const float maxSpeed = maxParticleSpeed();
        const float safeDt = kCflFraction * dx / std::max(maxSpeed, 1e-4f);
        const float subDt = std::min(remaining, safeDt);
        substep(params, subDt);
        remaining -= subDt;
        ++iterations;
    }
    if (remaining > 1e-8f) {
        // Safety-net fallback: kMaxSubsteps was exhausted without covering
        // the full requested dt (see kMaxSubsteps' comment — not expected in
        // normal operation). Advance the rest in one final step rather than
        // silently leaving the simulation behind the caller's clock.
        substep(params, remaining);
    }
}

} // namespace fluids::mpm
