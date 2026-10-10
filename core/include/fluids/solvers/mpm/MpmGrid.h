#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace fluids::mpm {

// A node-centered grid for MLS-MPM: node (i, j) sits at world position
// (i*dx, j*dx) and holds a single accumulated mass + velocity/momentum
// sample (unlike FLIP's MacGrid, which staggers separate u/v components on
// cell faces). This is a deliberately different layout from MacGrid — MPM's
// P2G/G2P quadratic-B-spline stencil is naturally node-centered (one 3x3
// neighborhood per particle, both velocity components transferred together
// via the same weights), whereas MacGrid's staggering exists specifically to
// make FLIP's pressure-projection Laplacian easy to write. Forcing MPM onto
// MacGrid (or vice versa) would fight both algorithms' natural math.
//
// `vx`/`vy` double as scratch: during P2G they accumulate momentum
// (mass * velocity + affine contribution), and updateGrid() divides by mass
// in place to turn them into actual velocities for the rest of the step.
struct MpmGrid {
    int nx = 0;
    int ny = 0;
    float dx = 0.0f;

    std::vector<float> mass; // size nx*ny
    std::vector<float> vx;   // size nx*ny — momentum during P2G, velocity after updateGrid()
    std::vector<float> vy;   // size nx*ny

    void resize(int nx_, int ny_, float dx_) {
        nx = nx_;
        ny = ny_;
        dx = dx_;
        mass.assign(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0.0f);
        vx.assign(mass.size(), 0.0f);
        vy.assign(mass.size(), 0.0f);
    }

    void clear() {
        std::fill(mass.begin(), mass.end(), 0.0f);
        std::fill(vx.begin(), vx.end(), 0.0f);
        std::fill(vy.begin(), vy.end(), 0.0f);
    }

    std::size_t index(int i, int j) const {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i);
    }

    bool inBounds(int i, int j) const { return i >= 0 && i < nx && j >= 0 && j < ny; }
};

} // namespace fluids::mpm
