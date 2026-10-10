#pragma once

#include <cstddef>

#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/sph_smoke/SphSmokeParams.h"
#include "fluids/solvers/sph_smoke/SphSmokeSolver.h"

namespace fluids {

// Public-facing SPH-flavored smoke simulation object bound to JS via Embind
// (see wasm/sph_smoke_bindings.cpp) — Phase 4c, the final slice of Phase 4
// (Smoke, added to all three solver families: FLIP/grid, MPM/APIC, and now
// SPH/kernel-particle). This is a sixth, independent concrete class
// alongside Simulation (SPH water), FlipSimulation (FLIP water),
// MpmSimulation (MPM water), FlipSmokeSimulation (Phase 4a, grid smoke), and
// MpmSmokeSimulation (Phase 4b, APIC particle smoke) — no shared polymorphic
// base, same reasoning as every prior addition in this project.
//
// SphSmokeSimulation reuses SPH water's actual kernel/neighbor-search
// machinery (fluids::UniformGrid, fluids::sph::poly6/viscosityLaplacian from
// Kernels.h) rather than a second copy of either other smoke solver's
// transfer scheme — see fluids::sph_smoke::SphSmokeSolver's class comment
// for the full rationale, including the deliberate no-pressure/EOS decision.
//
// JS contract shape: identical to MpmSmokeSimulation's particle contract
// (getParticleCount()/getPositionsPtr()/getVelocitiesPtr()/
// getDensitiesPtr()/getTemperaturesPtr(), same fixed-capacity/pointer-
// stability convention, same reuse of SmokeScenario/makePlumeScenario()
// unchanged) — see docs/sph-smoke-sim-api.md for the full writeup.
class SphSmokeSimulation {
public:
    SphSmokeSimulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state (to empty) AND (re)allocates the fixed-capacity
    // particle buffers from a scenario descriptor and the current
    // `params.maxParticles`. ALWAYS reallocates the buffers (capacity may
    // have changed since the last reset()) — callers holding a
    // pointer/typed-array view from getPositionsPtr()/getVelocitiesPtr()/
    // getDensitiesPtr()/getTemperaturesPtr() MUST re-fetch it after calling
    // reset(). Between reset() calls, the buffers are never reallocated, so
    // a pointer/view taken right after reset() stays valid across any
    // number of step() calls despite getParticleCount() changing value.
    void reset(const SmokeScenario& scenario);

    std::size_t getParticleCount() const;

    // Pointers to the start of the fixed-capacity, contiguous buffers. See
    // reset()'s and the class comment's buffer-stability contract.
    //   - positions/velocities: [x0, y0, x1, y1, ...], getParticleCount()*2
    //     floats.
    //   - densities/temperatures: [d0, d1, ...] / [t0, t1, ...],
    //     getParticleCount() floats (scalars, not vec2s).
    const float* getPositionsPtr() const;
    const float* getVelocitiesPtr() const;
    const float* getDensitiesPtr() const;
    const float* getTemperaturesPtr() const;

    // Live-tunable SPH smoke parameters. Public by design so JS can bind it
    // as a settable field once exposed as an Embind value_object (same
    // read-modify-write pattern gotcha as every other solver's params).
    sph_smoke::SphSmokeParams params;

private:
    sph_smoke::SphSmokeSolver solver_;
};

} // namespace fluids
