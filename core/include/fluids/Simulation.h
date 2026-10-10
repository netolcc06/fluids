#pragma once

#include <cstddef>
#include <cstdint>

#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/sph/SPHParams.h"
#include "fluids/solvers/sph/SPHSolver.h"

namespace fluids {

// Public-facing simulation object bound to JS via Embind (wasm/bindings.cpp)
// — this interface is meant to stay stable as later phases (FLIP, MPM) are
// added, so extending the project later doesn't break the JS contract.
//
// `Simulation` owns an SPHSolver by direct composition (not a polymorphic
// multi-solver dispatch layer) — deliberately, per the master plan: a
// dispatch abstraction is premature until a second solver family actually
// exists to abstract over.
class Simulation {
public:
    Simulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state from a scenario descriptor (see Scenario.h).
    // May change the particle count, which reallocates the position/
    // velocity buffers — callers holding a pointer/typed-array view from
    // getPositionsPtr()/getVelocitiesPtr() MUST re-fetch it after calling
    // reset(). Between reset() calls, the buffers are never reallocated, so
    // a pointer/view taken right after reset() stays valid across any
    // number of step() calls.
    void reset(const Scenario& scenario);

    std::size_t getParticleCount() const;

    // Pointers to the start of the contiguous position/velocity buffers,
    // each laid out as [x0, y0, x1, y1, ...] (getParticleCount() * 2
    // floats). See reset()'s comment for the buffer-stability contract.
    const float* getPositionsPtr() const;
    const float* getVelocitiesPtr() const;

    // Live-tunable WCSPH parameters, read fresh at the start of every
    // step(). Public by design so JS can bind it as a settable field once
    // exposed as an Embind value_object (see docs/sim-api.md for the
    // get-modify-set-back pattern this requires on the JS side).
    sph::SPHParams params;

private:
    sph::SPHSolver solver_;
};

} // namespace fluids
