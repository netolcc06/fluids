#pragma once

#include <cstddef>

#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/smoke/FlipSmokeParams.h"
#include "fluids/solvers/smoke/FlipSmokeSolver.h"

namespace fluids {

// Public-facing smoke simulation object bound to JS via Embind (see
// wasm/flip_smoke_bindings.cpp). This is a genuinely different algorithm
// from fluids::FlipSimulation (PIC/FLIP water), NOT a mode switch on it —
// smoke is a pure Eulerian (grid-only) density/temperature field advected
// on a MAC grid via the classic Stable Fluids method (Jos Stam), with no
// free surface and no particles at all. It's a third, independent concrete
// class alongside Simulation (SPH water) and FlipSimulation (FLIP water) —
// same reasoning as FlipSimulation's own header comment on why it isn't a
// shared polymorphic interface: premature abstraction with this few
// concrete solvers, all with genuinely different JS-facing shapes anyway
// (particle buffers for water, a grid-shaped density/temperature field for
// smoke — see below). fluids::FlipSimulation/flip::FlipSolver and
// core/solvers/flip/ are untouched by this addition; this class only
// *reuses* flip::MacGrid's data layout and Jacobi-projection style by
// including it, in a new fluids::smoke::FlipSmokeSolver.
//
// JS contract shape (deliberately different from water's
// getPositionsPtr()/getVelocitiesPtr()/getParticleCount(), since there are
// no particles to report a count or position/velocity buffer for):
//   - getGridWidth() / getGridHeight(): the MAC grid's cell resolution.
//   - getDensityFieldPtr(): pointer to a contiguous, row-major
//     width*height float array, one density value per cell, ~[0,1] range —
//     built for direct use as a renderable texture/canvas source (see
//     docs/flip-smoke-sim-api.md for the exact byte layout and a worked
//     example of turning this into a WebGL texture).
//   - getTemperatureFieldPtr(): same shape/layout, exposed too in case the
//     frontend wants to colorize by temperature (e.g. a hot-core / cool-
//     edge gradient) rather than density alone. Density alone is enough to
//     render "smoke," but temperature is cheap to expose (it's already a
//     first-class field internally) and gives the renderer a choice.
// Both pointers follow the exact same zero-copy Module.HEAPF32 pattern
// established for water's position/velocity buffers.
class FlipSmokeSimulation {
public:
    FlipSmokeSimulation();

    // Advances simulation time by dt (seconds), using the current `params`.
    void step(float dt);

    // Resets the density/temperature fields to empty/ambient AND
    // (re)allocates the MAC grid from a scenario descriptor and the current
    // `params.cellSize` — same reset()-owns-grid-allocation contract as
    // FlipSimulation::reset(). May change grid dimensions, which
    // reallocates the field buffers — callers holding a pointer/typed-array
    // view from getDensityFieldPtr()/getTemperatureFieldPtr() MUST re-fetch
    // it (and getGridWidth()/getGridHeight()) after calling reset().
    // Between reset() calls, the buffers are never reallocated, so a
    // pointer/view taken right after reset() stays valid across any number
    // of step() calls. Changing `params.cellSize` without calling reset()
    // afterward has no effect on the grid.
    void reset(const SmokeScenario& scenario);

    int getGridWidth() const;
    int getGridHeight() const;

    // Pointers to the start of the contiguous, row-major cell-centered
    // scalar field buffers, each getGridWidth()*getGridHeight() floats:
    // index j*getGridWidth()+i is cell (i, j). See reset()'s comment for
    // the buffer-stability contract.
    const float* getDensityFieldPtr() const;
    const float* getTemperatureFieldPtr() const;

    // Live-tunable Stable Fluids parameters. Public by design so JS can
    // bind it as a settable field once exposed as an Embind value_object —
    // see docs/flip-smoke-sim-api.md for the read-modify-write pattern this
    // requires (verified empirically for this build; same gotcha family as
    // sph::SPHParams/flip::FlipParams).
    smoke::FlipSmokeParams params;

private:
    smoke::FlipSmokeSolver solver_;
};

} // namespace fluids
