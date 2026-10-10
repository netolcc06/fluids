#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "fluids/common/Mat2.h"
#include "fluids/common/Vec2.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/mpm/MpmGrid.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeParams.h"

namespace fluids::mpm_smoke {

// MPM-flavored smoke: reuses MPM water's actual particle/APIC machinery
// (quadratic-B-spline P2G/G2P transfer, the APIC affine matrix C, the
// node-centered fluids::mpm::MpmGrid) instead of a second copy of Phase 4a's
// pure-Eulerian Stable Fluids grid solver (fluids::smoke::FlipSmokeSolver).
// This gives MPM smoke a genuinely different visual character — puffy,
// particle-driven — from FLIP smoke's smooth continuous field, and is the
// entire point of this being a separate Phase 4b slice rather than reusing
// Phase 4a's approach for a third time.
//
// Per-particle state: position, velocity, the APIC affine matrix C (2x2) —
// same as mpm::MpmSolver — but `density` (a smoke density scalar, ~[0,1],
// same semantics as FlipSmokeSolver's density field) and `temperature`
// (same semantics as FlipSmokeSolver's temperature field) replace water's
// `J` (volume ratio, only meaningful for an EOS pressure force). There is no
// pressure/stress force at all: smoke here is a compressible buoyant gas,
// not an incompressible fluid, so P2G's affine term is just `mass * C` (the
// APIC term alone) — dropping the `-dt*4/dx^2*volume*stress` term
// mpm::MpmSolver::particleToGrid adds for water. Per-particle mass is a
// fixed internal constant (see MpmSmokeSolver.cpp's kParticleMass), not
// calibrated from a rest density the way water's is, because nothing here
// depends on its absolute value — no stress term reads it, and it cancels
// out of the P2G/G2P momentum-weighted average.
//
// Continuous particle emission (new — no other solver in this project adds
// particles after reset()): each step, within SmokeScenario's source
// rectangle, new particles are spawned at a rate derived from
// SmokeScenario::emissionRate (see emit()'s comment in the .cpp for the
// exact translation and reasoning), seeded with density near 1.0,
// temperature near params.sourceTemperature, and small/zero initial
// velocity.
//
// Fixed-capacity particle buffer — IMPORTANT for the JS contract's
// pointer-stability convention (see docs/mpm-smoke-sim-api.md for the full
// writeup aimed at the frontend). Every other solver in this project
// guarantees position/velocity buffer pointers are stable across step()
// calls and only need refetching after reset(). Continuous emission would
// normally break that (particle count changes every step), so this solver
// preallocates every per-particle buffer at a fixed capacity
// (params.maxParticles) once in reset(), and only ever fills/uses the first
// activeCount() slots — the buffer POINTER never changes after reset() even
// though activeCount() legitimately changes every step as particles are
// emitted/culled. Callers must re-read particleCount() every step (cheap)
// and reslice their view to that many elements, but do NOT need to
// re-fetch the pointer itself except after reset().
//
// Particle culling: each step, after dissipation, any particle whose
// density has decayed below a small threshold (see MpmSmokeSolver.cpp's
// kDensityCullThreshold) is removed via swap-with-last-active-and-shrink
// (order among active particles is not preserved — nothing depends on it).
// Without this, the active count would only ever grow until it pinned at
// maxParticles and emission would silently stop forever. When at capacity,
// new emission is skipped for the rest of that tick rather than recycling
// the oldest/faintest particle in place — see emit()'s comment for why.
//
// Per-step pipeline (see .cpp for the derivation behind each stage),
// adapted from mpm::MpmSolver's P2G/grid-update/G2P shape with the stress
// term dropped and buoyancy/dissipation/emission/culling added:
//   1. Emit new particles into the active buffer.
//   2. Clear grid, P2G: scatter `mass*v + (mass*C)@dpos` (no stress term).
//   3. Grid update: momentum -> velocity, NO gravity (buoyancy replaces it,
//      same contract as FlipSmokeParams — there is no gravity field here),
//      margin-based boundary velocity clamp (same style as mpm::MpmSolver).
//   4. G2P: gather velocity + reconstruct C, same stencil as mpm::MpmSolver.
//   5. Buoyancy: v.y += dt * buoyancy * (temperature - ambientTemperature),
//      applied directly per particle using that particle's own temperature
//      (no grid-resident density/temperature fields are needed for this —
//      see .cpp for why that's the simpler choice here), plus optional
//      linear damping.
//   6. Advect: position += dt * v.
//   7. Dissipation: density decays multiplicatively, temperature relaxes
//      toward ambient — same formulas as FlipSmokeParams.
//   8. Cull faded particles, then a particle-level domain boundary clamp
//      (belt-and-suspenders, same pattern as every other solver).
//
// Stateless w.r.t. params, same convention as every other solver: step()
// takes MpmSmokeParams by const reference and reads it fresh every call,
// EXCEPT cellSize and maxParticles, which only take effect via reset()
// (buffer/grid (re)allocation only ever happens there).
//
// Empirically, plain fixed-substep stepping (no internal CFL-adaptive
// subdivision the way mpm::MpmSolver::step() needs for water's stiff EOS)
// was found stable across the caller dt range tested here — see
// docs/mpm-smoke-sim-api.md for the measurements. This solver has no
// stress/pressure feedback loop at all, so the specific runaway mode that
// forced water's adaptive substepping (a stiff EOS amplifying a large J
// swing at a sudden impact) cannot occur; the only remaining question was
// whether a fast particle stream could tunnel more than a cell per step,
// which was checked directly rather than assumed.
class MpmSmokeSolver {
public:
    // Allocates the fixed-capacity particle buffers (params.maxParticles)
    // and the MPM node grid from the scenario's domain size and
    // params.cellSize. Never called mid-step.
    void reset(const SmokeScenario& scenario, const MpmSmokeParams& params);
    void step(const MpmSmokeParams& params, float dt);

    // Number of currently-active particles — changes every step as
    // particles are emitted/culled (expected, see class comment). Always
    // <= the fixed capacity established at the last reset() (see
    // maxParticleCapacity()).
    std::size_t particleCount() const { return activeCount_; }

    // Buffers are sized to the fixed capacity established at reset() (see
    // maxParticleCapacity()) and NEVER reallocated between reset() calls —
    // only the first particleCount() entries hold meaningful/live data at
    // any given time. This backs the JS-facing pointer-stability contract;
    // see the class comment and docs/mpm-smoke-sim-api.md.
    const std::vector<Vec2>& positions() const { return positions_; }
    const std::vector<Vec2>& velocities() const { return velocities_; }
    const std::vector<float>& density() const { return density_; }
    const std::vector<float>& temperature() const { return temperature_; }

    // Fixed capacity established at the last reset() (== params.maxParticles
    // at that time). Read-only access for native tests/debugging.
    std::size_t maxParticleCapacity() const { return maxParticles_; }

    // Read-only grid access for native tests/debugging — not part of the JS
    // contract (only MpmSmokeSimulation is bound to Embind).
    const mpm::MpmGrid& grid() const { return grid_; }

    // Test-only helper: directly seeds one active particle with the given
    // state, bypassing normal source-region emission entirely. Returns
    // false (no-op) if the buffer is already at capacity. Used by native
    // tests to exercise culling behavior against a synthetic pre-seeded
    // batch without waiting on real emission/dissipation timing (see
    // tests/core/test_mpm_smoke_solver.cpp). NOT exposed via Embind — not
    // part of the JS contract.
    bool debugSpawnParticle(const Vec2& position, const Vec2& velocity, float density, float temperature);

private:
    SmokeScenario scenario_;

    // Fixed-capacity buffers, allocated exactly once per reset() at size
    // maxParticles_ (see class comment). Only indices [0, activeCount_) are
    // live; the rest hold stale/unused data left over from prior culls or
    // simply default-constructed values, never read.
    std::vector<Vec2> positions_;
    std::vector<Vec2> velocities_;
    std::vector<Mat2> C_;
    std::vector<float> density_;
    std::vector<float> temperature_;

    std::size_t activeCount_ = 0;
    std::size_t maxParticles_ = 0;

    mpm::MpmGrid grid_;

    std::mt19937 rng_;
    float emitAccumulator_ = 0.0f;

    void emit(const MpmSmokeParams& params, float dt);
    void particleToGrid();
    void updateGrid(const MpmSmokeParams& params);
    void gridToParticles();
    void applyBuoyancyAndDamping(const MpmSmokeParams& params, float dt);
    void advect(float dt);
    void applyDissipation(const MpmSmokeParams& params, float dt);
    void cullFaded();
    void clampToDomain();
};

} // namespace fluids::mpm_smoke
