#pragma once

#include <cstddef>
#include <cstdint>

#include "fluids/scenario/Scenario3D.h"
#include "fluids/solvers/sph3d/SphParams3D.h"
#include "fluids/solvers/sph3d/SphSolver3D.h"

namespace fluids {

// Public-facing 3D SPH water simulation object bound to JS via Embind (see
// wasm/sph3d_bindings.cpp) — the first 3D solver in the project, the 3D
// sibling of `Simulation` (2D SPH water, see Simulation.h). Same method
// shape and conventions as every 2D simulation class (own composition, no
// polymorphic multi-solver dispatch layer, same reasoning as the rest of the
// project), with one deliberate, prominent difference: positions and
// velocities are now 3 floats per particle, not 2.
//
// *** getParticleCount() * 3 floats per buffer, NOT * 2 — every 2D solver's
// JS contract uses count*2; this one does not. See docs/sph3d-sim-api.md. ***
class SphSimulation3D {
public:
    SphSimulation3D();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets particle state from a scenario descriptor (see Scenario3D.h).
    // May change the particle count, which reallocates the position/
    // velocity buffers — callers holding a pointer/typed-array view from
    // getPositionsPtr()/getVelocitiesPtr() MUST re-fetch it after calling
    // reset(). Between reset() calls, the buffers are never reallocated, so
    // a pointer/view taken right after reset() stays valid across any
    // number of step() calls.
    void reset(const Scenario3D& scenario);

    std::size_t getParticleCount() const;

    // Pointers to the start of the contiguous position/velocity buffers,
    // each laid out as [x0, y0, z0, x1, y1, z1, ...] — getParticleCount()*3
    // floats (three per particle, not two — see the class comment above and
    // docs/sph3d-sim-api.md). See reset()'s comment for the buffer-stability
    // contract.
    const float* getPositionsPtr() const;
    const float* getVelocitiesPtr() const;

    // Live-tunable 3D WCSPH parameters, read fresh at the start of every
    // step(). Public by design so JS can bind it as a settable field once
    // exposed as an Embind value_object (same read-modify-write pattern
    // gotcha as the 2D solver's params — see docs/sph3d-sim-api.md).
    sph3d::SphParams3D params;

private:
    sph3d::SphSolver3D solver_;
};

} // namespace fluids
