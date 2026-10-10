#pragma once

#include <cstddef>
#include <vector>

#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/flip/MacGrid.h"
#include "fluids/solvers/smoke/FlipSmokeParams.h"

namespace fluids::smoke {

// Classic Stable Fluids (Jos Stam) 2D grid-only smoke solver, built on
// fluids::flip::MacGrid — the same staggered velocity layout (u on vertical
// faces, v on horizontal faces) and the same style of fixed-iteration
// Jacobi pressure projection as fluids::flip::FlipSolver (water), because
// both problems really do share that piece: velocity lives on a MAC grid
// and incompressibility comes from the same kind of Poisson solve either
// way. Everything else is different on purpose (see class-level comment in
// FlipSmokeSimulation.h for the full "why a separate class" rationale):
//
//   - No particles at all. Density and temperature are pure Eulerian
//     scalar fields, one value per cell, advected directly on the grid.
//   - No marker-and-cell classification from particle occupancy. Cell
//     classification is static and computed once in reset(): every
//     interior cell is CellType::Fluid ("air that can flow"), the
//     one-cell-thick outer ring is CellType::Solid — there is no
//     CellType::Air state for smoke at all (every non-Solid cell always
//     carries a legitimate, if possibly zero, density/temperature value).
//   - No gravity. Buoyancy (proportional to temperature above ambient) is
//     the sole driving force, applied directly to v-face velocities.
//   - Velocity is self-advected (semi-Lagrangian) every step instead of
//     being carried by particles — there is no P2G/G2P transfer at all.
//
// Per-step pipeline (see .cpp for the derivation behind each stage):
//   1. Emit source: add density and blend temperature toward
//      params.sourceTemperature within scenario_'s source region.
//   2. Buoyancy: v-face velocities += dt * buoyancy * (T_face - ambient).
//   3. Pressure projection: same fixed-iteration Jacobi scheme as
//      FlipSolver, adapted for the simpler Fluid/Solid-only
//      classification (see projectPressure()'s comment for exactly what
//      changes vs. water's version). Zero solid-adjacent face velocities
//      both before and after, same principle as FLIP water.
//   4. Self-advect velocity: semi-Lagrangian backward trace of each face
//      through the current (post-projection) velocity field, sampling the
//      OLD velocity field there.
//   5. Advect density/temperature: same semi-Lagrangian technique, tracing
//      through the SAME post-projection velocity snapshot used in step 4
//      (not the just-computed step-4 output — see .cpp), sampling the old
//      density/temperature fields.
//   6. Dissipation: density decays multiplicatively; temperature relaxes
//      toward ambient.
//   7. Boundary: zero solid-adjacent face velocities once more (advection
//      can reintroduce a nonzero normal component right at the boundary).
//
// Stateless w.r.t. params, same convention as FlipSolver: step() takes
// FlipSmokeParams by const reference and reads it fresh every call, EXCEPT
// cellSize, which only takes effect via reset() (grid (re)allocation
// happens there only).
class FlipSmokeSolver {
public:
    // Allocates the MAC grid + density/temperature fields from the
    // scenario's domain size and params.cellSize, and (re)classifies cells
    // (static for smoke — see class comment). Never called mid-step.
    void reset(const SmokeScenario& scenario, const FlipSmokeParams& params);
    void step(const FlipSmokeParams& params, float dt);

    int gridWidth() const { return grid_.nx; }
    int gridHeight() const { return grid_.ny; }

    // Cell-centered scalar fields, row-major, size gridWidth()*gridHeight()
    // each: index j*gridWidth()+i is cell (i, j) (same row-major convention
    // as MacGrid's own cell-centered arrays, e.g. pressure/cellType). Not
    // reallocated between reset() calls — see FlipSmokeSimulation for the
    // JS-facing pointer-stability contract this backs.
    const std::vector<float>& density() const { return density_; }
    const std::vector<float>& temperature() const { return temperature_; }

    // Read-only grid access for native tests/debugging (e.g. checking
    // post-projection divergence, cell classification). Not part of the JS
    // contract — only FlipSmokeSimulation is bound to Embind.
    const flip::MacGrid& grid() const { return grid_; }

private:
    SmokeScenario scenario_;
    flip::MacGrid grid_;
    std::vector<float> density_;      // size nx*ny, cell-centered, ~[0,1]
    std::vector<float> temperature_;  // size nx*ny, cell-centered, relative scale

    void classifyCells();
    void emitSource(const FlipSmokeParams& params, float dt);
    void applyBuoyancy(const FlipSmokeParams& params, float dt);
    void enforceSolidFaces();
    void projectPressure();
    void advect(float dt);
    void applyDissipation(const FlipSmokeParams& params, float dt);
};

} // namespace fluids::smoke
