#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "fluids/common/UniformGrid.h"
#include "fluids/common/Vec2.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/sph_smoke/SphSmokeParams.h"

namespace fluids::sph_smoke {

// SPH-flavored smoke: reuses SPH water's actual kernel/neighbor-search
// machinery (fluids::UniformGrid for O(1)-average neighbor queries, and
// fluids::sph::poly6 / fluids::sph::viscosityLaplacian from
// fluids/solvers/sph/Kernels.h, completely unmodified) instead of a second
// copy of Phase 4a's pure-Eulerian grid solver or Phase 4b's MPM/APIC
// transfer. This gives SPH smoke a genuinely different character again: a
// buoyant particle gas whose only inter-particle coupling is direct
// kernel-weighted neighbor averaging (no grid of any kind mediates it, unlike
// both other smoke variants) — the entire point of this being a third,
// separate Phase 4c slice.
//
// Per-particle state: position, velocity, `density` (a smoke density scalar,
// ~[0,1], same semantics as the other two smoke variants' density field) and
// `temperature` (same semantics as the other two). This is deliberately NOT
// the same thing as the internal SPH kernel-density estimate computed each
// step for the viscosity term (see sphDensity() below) — see the "Pressure/
// EOS decision" section of docs/sph-smoke-sim-api.md for why those two
// notions of "density" are kept conceptually and numerically separate here.
//
// **No pressure/EOS force at all** (see docs/sph-smoke-sim-api.md's
// "Pressure/EOS decision" section for the full empirical writeup this
// decision is based on). Water's WCSPH calibrates a stiff Tait
// equation-of-state against a roughly rest-packed local particle density;
// smoke's particle population is fundamentally different — continuously
// emitted, spreading and thinning as it rises, with no meaningful "rest
// density" to calibrate against. This solver's only force terms are
// buoyancy (a direct per-particle body force, not mediated by any kernel)
// and a kernel-weighted Laplacian viscosity (reusing
// fluids::sph::viscosityLaplacian unchanged) — the latter supplies this
// solver's "genuine SPH character" (local averaging/diffusion between
// neighbors) entirely on its own, with no pressure term needed for either
// coherence or stability. This also means the water SPH near-coincidence
// force-spike bug (see fluids::sph::Kernels.h's kMinEffectiveRFraction
// comment) structurally cannot occur here: that bug's mechanism was
// specifically the Tait EOS's exponent-7 amplification of an inflated
// poly6-density estimate near r=0, funneled through spikyGradient's
// pressure force — this solver calls neither spikyGradient nor any EOS, so
// there is no exponent-amplifying feedback loop for a near-coincident pair
// to trigger in the first place. (viscosityLaplacian itself is a plain,
// smooth, LINEAR-in-r function with a small finite maximum at r=0 — nothing
// here to floor or re-derive.)
//
// Continuous particle emission (same shape as MpmSmokeSolver, see its class
// comment): each step, within SmokeScenario's source rectangle, new
// particles are spawned at a rate derived from SmokeScenario::emissionRate
// (see emit()'s comment in the .cpp — same translation formula
// MpmSmokeSolver uses), seeded with density/temperature near their source
// values and a small random velocity jitter.
//
// Fixed-capacity particle buffer — same JS pointer-stability convention as
// MpmSmokeSolver (see docs/sph-smoke-sim-api.md for the full writeup aimed
// at the frontend): every per-particle buffer is preallocated at a fixed
// capacity (params.maxParticles) once in reset(), and only the first
// particleCount() slots are meaningful at any given moment — buffers are
// never reallocated between reset() calls, so a pointer/view taken right
// after reset() stays valid across any number of step() calls despite
// particleCount() legitimately changing every step.
//
// Particle culling: same shape as MpmSmokeSolver — each step, after
// dissipation, any particle whose density has decayed below a small
// threshold is removed via swap-with-last-active-and-shrink. When at
// capacity, new emission is skipped for the rest of that tick rather than
// recycling the oldest/faintest particle in place — same reasoning as
// MpmSmokeSolver's emit().
//
// Per-step pipeline (see .cpp for the derivation behind each stage):
//   1. Emit new particles into the active buffer.
//   2. Copy active positions into a scratch buffer and build a UniformGrid
//      over them, keyed by params.smoothingRadius (same neighbor-search
//      approach as sph::SPHSolver, rebuilt every step since positions and
//      smoothingRadius both change/are live-tunable).
//   3. Compute a per-particle SPH kernel-density estimate (poly6 sum) —
//      used ONLY to normalize the viscosity term below, not exposed to JS
//      and not the same value as the particle's own `density` state
//      variable (see class comment above).
//   4. Compute per-particle acceleration: buoyancy (direct body force) plus
//      kernel-weighted Laplacian viscosity (diffuses relative velocity
//      toward neighbors, normalized by the kernel-density estimate from
//      step 3 — see the .cpp for why this normalization can never divide by
//      a near-zero value here, unlike a naive port of water's formula might
//      suggest).
//   5. Integrate: symplectic Euler, same as sph::SPHSolver.
//   6. Dissipation: density decays multiplicatively, temperature relaxes
//      toward ambient — same formulas as every other smoke variant.
//   7. Cull faded particles, then a particle-level domain boundary clamp
//      (belt-and-suspenders, same pattern as every other solver — see the
//      .cpp for why this solver, like water SPH, needs only this one layer
//      and no grid-level boundary step).
//
// Stateless w.r.t. params, same convention as every other solver: step()
// takes SphSmokeParams by const reference and reads it fresh every call,
// EXCEPT maxParticles, which only takes effect via reset() (buffer
// (re)allocation only ever happens there). Unlike cellSize in the two
// grid/APIC-based smoke solvers, smoothingRadius does NOT require a reset()
// to take effect — the UniformGrid is rebuilt from scratch every step
// regardless (same as water SPH), so a live change is picked up on the very
// next step.
class SphSmokeSolver {
public:
    // Allocates the fixed-capacity particle buffers (params.maxParticles).
    // Never called mid-step.
    void reset(const SmokeScenario& scenario, const SphSmokeParams& params);
    void step(const SphSmokeParams& params, float dt);

    // Number of currently-active particles — changes every step as
    // particles are emitted/culled (expected, see class comment). Always
    // <= the fixed capacity established at the last reset() (see
    // maxParticleCapacity()).
    std::size_t particleCount() const { return activeCount_; }

    // Buffers are sized to the fixed capacity established at reset() (see
    // maxParticleCapacity()) and NEVER reallocated between reset() calls —
    // only the first particleCount() entries hold meaningful/live data at
    // any given time. This backs the JS-facing pointer-stability contract.
    const std::vector<Vec2>& positions() const { return positions_; }
    const std::vector<Vec2>& velocities() const { return velocities_; }
    const std::vector<float>& density() const { return density_; }
    const std::vector<float>& temperature() const { return temperature_; }

    // Fixed capacity established at the last reset() (== params.maxParticles
    // at that time). Read-only access for native tests/debugging.
    std::size_t maxParticleCapacity() const { return maxParticles_; }

    // Read-only access to the internal SPH kernel-density estimate computed
    // during the most recent step() (see class comment — NOT the same thing
    // as density()). Only entries [0, particleCount()) are meaningful. Not
    // part of the JS contract (only SphSmokeSimulation is bound to Embind) —
    // exposed for native tests/diagnostics validating the "never divides by
    // a near-zero density" claim above.
    const std::vector<float>& sphDensity() const { return sphDensity_; }

    // Test-only helper: directly seeds one active particle with the given
    // state, bypassing normal source-region emission entirely. Returns
    // false (no-op) if the buffer is already at capacity. Same role as
    // MpmSmokeSolver::debugSpawnParticle — used by native tests to exercise
    // culling behavior against a synthetic pre-seeded batch. NOT exposed via
    // Embind.
    bool debugSpawnParticle(const Vec2& position, const Vec2& velocity, float density, float temperature);

private:
    SmokeScenario scenario_;

    // Fixed-capacity buffers, allocated exactly once per reset() at size
    // maxParticles_ (see class comment). Only indices [0, activeCount_) are
    // live; the rest hold stale/unused data left over from prior culls or
    // simply default-constructed values, never read.
    std::vector<Vec2> positions_;
    std::vector<Vec2> velocities_;
    std::vector<float> density_;
    std::vector<float> temperature_;

    // Per-step scratch buffers, resized as needed — not part of the
    // pointer-stability contract (never exposed to JS).
    std::vector<Vec2> accelerationsScratch_;
    std::vector<float> sphDensity_;
    std::vector<Vec2> activePositionsScratch_;
    mutable std::vector<int> neighborScratch_;

    std::size_t activeCount_ = 0;
    std::size_t maxParticles_ = 0;

    std::mt19937 rng_;
    float emitAccumulator_ = 0.0f;

    void emit(const SphSmokeParams& params, float dt);
    void computeSphDensity(const SphSmokeParams& params, const UniformGrid& grid);
    void computeForces(const SphSmokeParams& params, const UniformGrid& grid);
    void integrate(float dt);
    void applyDissipation(const SphSmokeParams& params, float dt);
    void cullFaded();
    void clampToDomain();
};

} // namespace fluids::sph_smoke
