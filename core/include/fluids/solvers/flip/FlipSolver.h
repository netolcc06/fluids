#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "fluids/common/Vec2.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/flip/FlipParams.h"
#include "fluids/solvers/flip/MacGrid.h"

namespace fluids::flip {

// Bridson-style PIC/FLIP water solver on a MAC grid.
//
// Per-step pipeline (see .cpp for the derivation behind each stage):
//   1. P2G: bilinear-weighted scatter of particle velocities onto grid
//      faces, normalized by accumulated weight.
//   2. Classify cells (Fluid/Air/Solid) from current particle positions.
//   3. Apply gravity directly to grid velocities.
//   4. Pressure projection: Jacobi iteration on the fluid-cell Poisson
//      equation (fixed iteration count — see kPressureIterations in the
//      .cpp for why Jacobi-over-CG was chosen for Phase 2), then zero
//      solid-adjacent face velocities.
//   5. Extrapolate valid velocities a few cells into neighboring Air cells
//      (both the post-projection field AND the pre-projection P2G
//      snapshot, using the same validity mask — see gridToParticles()) so
//      G2P never samples a stale/zero air-cell value.
//   6. G2P: PIC/FLIP blend per particle.
//   7. Forward-Euler advection.
//   8. Clamp any particle that escaped [0,domainWidth]x[0,domainHeight]
//      back onto the boundary (belt-and-suspenders on top of #4's solid
//      handling).
//
// SPHSolver-style statelessness w.r.t. params: step() takes FlipParams by
// const reference and reads it fresh every call, EXCEPT cellSize, which
// only takes effect via reset() (grid (re)allocation happens there only —
// see FlipParams::cellSize).
class FlipSolver {
public:
    // reset() takes FlipParams (unlike SPHSolver::reset(Scenario)) because
    // the MAC grid's resolution depends on params.cellSize; grid
    // (re)allocation only ever happens here, never mid-step. This is a
    // solver-internal signature difference only — it does not affect the
    // JS-facing contract, which is entirely defined by FlipSimulation
    // (reset(Scenario) only, params supplied separately as a settable
    // property, same shape as the SPH Simulation class).
    void reset(const Scenario& scenario, const FlipParams& params);
    void step(const FlipParams& params, float dt);

    std::size_t particleCount() const { return positions_.size(); }
    const std::vector<Vec2>& positions() const { return positions_; }
    const std::vector<Vec2>& velocities() const { return velocities_; }

    // Read-only grid access for native tests/debugging (e.g. checking
    // post-projection divergence). Not part of the JS contract — only
    // FlipSimulation is bound to Embind, and it does not expose this.
    const MacGrid& grid() const { return grid_; }

private:
    Scenario scenario_;
    std::vector<Vec2> positions_;
    std::vector<Vec2> velocities_;

    MacGrid grid_;
    // Snapshot of grid_.u/v taken right after P2G (before gravity/
    // projection), used to compute the FLIP "change in grid velocity"
    // delta during G2P.
    std::vector<float> uOld_;
    std::vector<float> vOld_;
    // Which faces received at least one P2G contribution this step (before
    // gravity/projection touch them). Reused to extrapolate both the old
    // snapshot and the new post-projection field with an identical mask.
    std::vector<std::uint8_t> uValid_;
    std::vector<std::uint8_t> vValid_;

    void seedRegion(float x, float y, float width, float height, float vx, float vy);

    void particleToGrid();
    void classifyCells();
    void applyGravity(float gravity, float dt);
    void enforceSolidFaces();
    void projectPressure();
    static void extrapolate(std::vector<float>& field, const std::vector<std::uint8_t>& validMask, int w, int h,
                             int depth);
    void gridToParticles(float flipRatio);
    void advect(float dt);
    void clampToDomain();
};

} // namespace fluids::flip
