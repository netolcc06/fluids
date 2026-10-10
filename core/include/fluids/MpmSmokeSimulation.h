#pragma once

#include <cstddef>

#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeParams.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeSolver.h"

namespace fluids {

// Public-facing MPM smoke simulation object bound to JS via Embind (see
// wasm/mpm_smoke_bindings.cpp) — Phase 4b. This is a genuinely different
// algorithm from both fluids::MpmSimulation (MPM water: APIC transfer PLUS
// an EOS pressure/stress force, tracking J) and fluids::FlipSmokeSimulation
// (Phase 4a: a pure-Eulerian grid-only density/temperature field, no
// particles at all). MPM smoke reuses MPM water's actual particle/APIC
// P2G/G2P transfer machinery, but drops the pressure/stress force entirely
// and tracks a per-particle density/temperature instead of a volume ratio
// J — see fluids::mpm_smoke::MpmSmokeSolver's class comment for the full
// rationale. It is a fifth, independent concrete class alongside Simulation
// (SPH water), FlipSimulation (FLIP water), MpmSimulation (MPM water), and
// FlipSmokeSimulation (FLIP/grid smoke) — no shared polymorphic base, same
// reasoning as every prior addition in this project.
//
// JS contract shape: mirrors MpmSimulation's particle contract
// (getParticleCount()/getPositionsPtr()/getVelocitiesPtr()) PLUS
// getDensitiesPtr()/getTemperaturesPtr() for the per-particle scalars this
// solver tracks instead of water's J — contiguous float32 arrays of length
// getParticleCount() (not *2 — these are scalars, not vec2s). It reuses
// SmokeScenario unchanged (the same struct FlipSmokeSimulation uses) rather
// than water's Scenario, since it needs a continuously-emitting source
// region, not a one-shot particle seed — see SmokeScenario.h.
//
// IMPORTANT pointer-stability nuance (see MpmSmokeSolver's class comment and
// docs/mpm-smoke-sim-api.md for the full writeup): because particles are
// continuously emitted and culled, getParticleCount() legitimately changes
// value every single step() call — that's normal and expected, same as it
// already changes across different scenarios' reset() calls for every other
// solver. What does NOT change across step() calls is the underlying
// buffer POINTER: all per-particle buffers are preallocated at a fixed
// capacity (params.maxParticles) in reset() and never reallocated
// afterward, so getPositionsPtr()/getVelocitiesPtr()/getDensitiesPtr()/
// getTemperaturesPtr() all return the SAME address across any number of
// step() calls, only needing to be re-fetched after reset() (same
// discipline as every other solver). Callers must re-read
// getParticleCount() every frame (cheap) and reslice their typed-array view
// to that many elements.
class MpmSmokeSimulation {
public:
    MpmSmokeSimulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state (to empty) AND (re)allocates the fixed-capacity
    // particle buffers and the MPM node grid from a scenario descriptor and
    // the current `params.cellSize`/`params.maxParticles`. ALWAYS
    // reallocates the buffers (capacity may have changed since the last
    // reset()) — callers holding a pointer/typed-array view from
    // getPositionsPtr()/getVelocitiesPtr()/getDensitiesPtr()/
    // getTemperaturesPtr() MUST re-fetch it after calling reset(). Between
    // reset() calls, the buffers are never reallocated (see class comment),
    // so a pointer/view taken right after reset() stays valid across any
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

    // Live-tunable MPM smoke parameters. Public by design so JS can bind it
    // as a settable field once exposed as an Embind value_object (see
    // docs/mpm-smoke-sim-api.md for the read-modify-write pattern this
    // requires — the same gotcha as every other solver's params).
    mpm_smoke::MpmSmokeParams params;

private:
    mpm_smoke::MpmSmokeSolver solver_;
};

} // namespace fluids
