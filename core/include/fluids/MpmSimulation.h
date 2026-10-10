#pragma once

#include <cstddef>
#include <cstdint>

#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/mpm/MpmParams.h"
#include "fluids/solvers/mpm/MpmSolver.h"

namespace fluids {

// Public-facing MPM simulation object bound to JS via Embind (see
// wasm/mpm_bindings.cpp) — mirrors fluids::Simulation (SPH) and
// fluids::FlipSimulation's method shape exactly (step/reset/
// getParticleCount/getPositionsPtr/getVelocitiesPtr, public `params`) so the
// frontend can drive all three solvers uniformly. This is a third,
// independent concrete class — no shared polymorphic base with Simulation
// or FlipSimulation, same reasoning as FLIP's addition: three data points
// still isn't a strong enough signal to force an abstraction that has no
// other user yet. Simulation/SPH and FlipSimulation/FLIP are untouched by
// this addition.
class MpmSimulation {
public:
    MpmSimulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state AND (re)allocates the MPM node grid from a
    // scenario descriptor (see Scenario.h) and the current
    // `params.cellSize`/`params.restDensity` (particle mass is calibrated
    // here too — see MpmParams.h). May change the particle count, which
    // reallocates the position/velocity buffers — callers holding a
    // pointer/typed-array view from getPositionsPtr()/getVelocitiesPtr()
    // MUST re-fetch it after calling reset(). Between reset() calls, the
    // buffers are never reallocated, so a pointer/view taken right after
    // reset() stays valid across any number of step() calls. Changing
    // `params.cellSize` (or `params.restDensity`) without calling reset()
    // afterward has no effect on the grid/particle-mass calibration.
    void reset(const Scenario& scenario);

    std::size_t getParticleCount() const;

    // Pointers to the start of the contiguous position/velocity buffers,
    // each laid out as [x0, y0, x1, y1, ...] (getParticleCount() * 2
    // floats). See reset()'s comment for the buffer-stability contract.
    const float* getPositionsPtr() const;
    const float* getVelocitiesPtr() const;

    // Live-tunable MLS-MPM parameters. Public by design so JS can bind it
    // as a settable field once exposed as an Embind value_object (see
    // docs/mpm-sim-api.md for the get-modify-set-back pattern this
    // requires on the JS side — the same gotcha as SPHParams/FlipParams).
    mpm::MpmParams params;

private:
    mpm::MpmSolver solver_;
};

} // namespace fluids
