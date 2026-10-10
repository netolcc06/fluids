#pragma once

#include <cstddef>
#include <vector>

#include "fluids/common/UniformGrid3D.h"
#include "fluids/common/Vec3.h"
#include "fluids/scenario/Scenario3D.h"
#include "fluids/solvers/sph3d/SphParams3D.h"

namespace fluids::sph3d {

// Weakly-Compressible SPH (WCSPH) water solver, 3D sibling of
// fluids::sph::SPHSolver (see SPHSolver.h) — same algorithm, structure, and
// conventions, extended to three dimensions.
//
// - Density estimation: 3D Poly6 kernel (Kernels3D.h).
// - Pressure force: symmetric Monaghan-style gradient using the 3D Spiky
//   kernel, from a Tait equation-of-state pressure (exponent 7, same as 2D).
// - Viscosity: standard Laplacian viscosity (Muller et al. 2003), not XSPH —
//   same rationale as 2D.
// - Neighbor search: UniformGrid3D keyed by the current smoothing radius,
//   rebuilt every step.
// - Boundary handling: clamp-to-box with reflected, damped velocity at each
//   of the 6 walls (min/max on x, y, and z) — the 3D extension of the 2D
//   solver's 4-wall clamped-reflect scheme, same SphParams3D::boundaryDamping
//   coefficient.
//
// SphSolver3D is stateless with respect to SphParams3D: every step() call
// takes params by const reference and reads it fresh, so live JS parameter
// edits take effect on the very next step with no separate "apply" call.
class SphSolver3D {
public:
    void reset(const Scenario3D& scenario);
    void step(const SphParams3D& params, float dt);

    std::size_t particleCount() const { return positions_.size(); }
    const std::vector<Vec3>& positions() const { return positions_; }
    const std::vector<Vec3>& velocities() const { return velocities_; }

private:
    Scenario3D scenario_;

    std::vector<Vec3> positions_;
    std::vector<Vec3> velocities_;
    std::vector<Vec3> accelerations_;
    std::vector<float> densities_;
    std::vector<float> pressures_;

    // Reused across steps to avoid reallocating on every neighbor query.
    mutable std::vector<int> neighborScratch_;

    void seedRegion(float x, float y, float z, float width, float height, float depth, float vx, float vy,
                     float vz);
    void computeDensityPressure(const SphParams3D& params, const UniformGrid3D& grid);
    void computeForces(const SphParams3D& params, const UniformGrid3D& grid);
    void integrate(const SphParams3D& params, float dt);
    void enforceBoundaries(const SphParams3D& params);
};

} // namespace fluids::sph3d
