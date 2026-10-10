#pragma once

#include <cstddef>
#include <vector>

#include "fluids/common/UniformGrid.h"
#include "fluids/common/Vec2.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/sph/SPHParams.h"

namespace fluids::sph {

// Weakly-Compressible SPH (WCSPH) water solver.
//
// - Density estimation: Poly6 kernel.
// - Pressure force: symmetric Monaghan-style gradient using the Spiky
//   kernel, from a Tait equation-of-state pressure (exponent 7).
// - Viscosity: standard Laplacian viscosity (Müller et al. 2003), not XSPH
//   — chosen because it's a direct force-space term that composes simply
//   with the pressure/gravity accumulation below, and its single
//   `viscosity` coefficient maps directly onto SPHParams::viscosity.
// - Neighbor search: UniformGrid keyed by the current smoothing radius,
//   rebuilt every step (positions change every step, and smoothingRadius
//   is itself live-tunable).
// - Boundary handling: clamp-to-box with reflected, damped velocity at each
//   wall (SPHParams::boundaryDamping). This is the simplest option and the
//   one the master plan calls out to start with; if it looks visually wrong
//   once rendered (e.g. particles sticking to walls, unnatural bounce), the
//   documented fallback is penalty-force boundaries (a soft repulsive force
//   near walls instead of a hard position clamp) — not implemented here to
//   avoid building two boundary schemes before either has been visually
//   validated.
//
// SPHSolver is stateless with respect to SPHParams: every step() call takes
// params by const reference and reads it fresh, so live JS parameter edits
// take effect on the very next step with no separate "apply" call.
class SPHSolver {
public:
    void reset(const Scenario& scenario);
    void step(const SPHParams& params, float dt);

    std::size_t particleCount() const { return positions_.size(); }
    const std::vector<Vec2>& positions() const { return positions_; }
    const std::vector<Vec2>& velocities() const { return velocities_; }

private:
    Scenario scenario_;

    std::vector<Vec2> positions_;
    std::vector<Vec2> velocities_;
    std::vector<Vec2> accelerations_;
    std::vector<float> densities_;
    std::vector<float> pressures_;

    // Reused across steps to avoid reallocating on every neighbor query.
    mutable std::vector<int> neighborScratch_;

    void seedRegion(float x, float y, float width, float height, float vx, float vy);
    void computeDensityPressure(const SPHParams& params, const UniformGrid& grid);
    void computeForces(const SPHParams& params, const UniformGrid& grid);
    void integrate(const SPHParams& params, float dt);
    void enforceBoundaries(const SPHParams& params);
};

} // namespace fluids::sph
