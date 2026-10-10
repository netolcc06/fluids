#pragma once

#include <cstddef>
#include <cstdint>

#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/flip/FlipParams.h"
#include "fluids/solvers/flip/FlipSolver.h"

namespace fluids {

// Public-facing FLIP simulation object bound to JS via Embind (see
// wasm/flip_bindings.cpp) — mirrors fluids::Simulation's method shape
// exactly (step/reset/getParticleCount/getPositionsPtr/getVelocitiesPtr,
// public `params`) so the frontend can drive both solvers uniformly. This
// is a second, independent concrete class, NOT a shared polymorphic
// interface with Simulation — deliberately, per the master plan: an
// abstraction is premature with only two solver families, before a third
// (MPM) exists to justify factoring one out. `Simulation`/SPH is untouched
// by this addition.
class FlipSimulation {
public:
    FlipSimulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state AND (re)allocates the MAC grid from a scenario
    // descriptor (see Scenario.h) and the current `params.cellSize`. May
    // change the particle count, which reallocates the position/velocity
    // buffers — callers holding a pointer/typed-array view from
    // getPositionsPtr()/getVelocitiesPtr() MUST re-fetch it after calling
    // reset(). Between reset() calls, the buffers are never reallocated, so
    // a pointer/view taken right after reset() stays valid across any
    // number of step() calls. Changing `params.cellSize` without calling
    // reset() afterward has no effect on the grid.
    void reset(const Scenario& scenario);

    std::size_t getParticleCount() const;

    // Pointers to the start of the contiguous position/velocity buffers,
    // each laid out as [x0, y0, x1, y1, ...] (getParticleCount() * 2
    // floats). See reset()'s comment for the buffer-stability contract.
    const float* getPositionsPtr() const;
    const float* getVelocitiesPtr() const;

    // Live-tunable PIC/FLIP parameters. Public by design so JS can bind it
    // as a settable field once exposed as an Embind value_object (see
    // docs/flip-sim-api.md for the get-modify-set-back pattern this
    // requires on the JS side — the same gotcha as sph::SPHParams).
    flip::FlipParams params;

private:
    flip::FlipSolver solver_;
};

} // namespace fluids
