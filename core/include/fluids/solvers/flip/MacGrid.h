#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fluids::flip {

// Marker-and-cell classification, redone from scratch every step from
// current particle positions (see FlipSolver::classifyCells).
enum class CellType : std::uint8_t { Air, Fluid, Solid };

// A 2D MAC (staggered) grid: horizontal velocity `u` lives on vertical cell
// faces ((nx+1) x ny samples), vertical velocity `v` lives on horizontal
// cell faces (nx x (ny+1) samples), and per-cell scalars (cellType,
// pressure) live at cell centers (nx x ny). Cell (i, j) covers
// [i*h, (i+1)*h] x [j*h, (j+1)*h]; face u(i, j) sits at x = i*h,
// y = (j+0.5)*h; face v(i, j) sits at x = (i+0.5)*h, y = j*h.
//
// Purely a data container + index math, like Vec2 — no behavior here is
// complex enough to warrant hiding in a .cpp. All the transfer/
// classification/projection logic lives in FlipSolver.
struct MacGrid {
    int nx = 0;
    int ny = 0;
    float h = 0.0f;

    std::vector<float> u;            // size (nx+1)*ny
    std::vector<float> v;            // size nx*(ny+1)
    std::vector<float> uWeight;      // same size as u — P2G accumulation weights
    std::vector<float> vWeight;      // same size as v
    std::vector<CellType> cellType;  // size nx*ny
    std::vector<float> pressure;     // size nx*ny — Jacobi scratch ("q"), see FlipSolver::projectPressure

    // (Re)allocates every buffer for a grid of the given resolution/cell
    // size, filled with zeros / CellType::Air. Called only from
    // FlipSolver::reset() — never mid-step.
    void resize(int nx_, int ny_, float h_) {
        nx = nx_;
        ny = ny_;
        h = h_;
        u.assign(static_cast<std::size_t>(nx + 1) * static_cast<std::size_t>(ny), 0.0f);
        v.assign(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny + 1), 0.0f);
        uWeight.assign(u.size(), 0.0f);
        vWeight.assign(v.size(), 0.0f);
        cellType.assign(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), CellType::Air);
        pressure.assign(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0.0f);
    }

    // Zeros velocity + weight accumulators at the start of each step's P2G
    // pass. Does not touch cellType/pressure (recomputed independently).
    void clearVelocities() {
        std::fill(u.begin(), u.end(), 0.0f);
        std::fill(v.begin(), v.end(), 0.0f);
        std::fill(uWeight.begin(), uWeight.end(), 0.0f);
        std::fill(vWeight.begin(), vWeight.end(), 0.0f);
    }

    // Face/cell index helpers. No bounds checking on (i, j) themselves
    // (callers stay within [0,nx]x[0,ny) for U, [0,nx)x[0,ny] for V,
    // [0,nx)x[0,ny) for cell-centered arrays) — use cellTypeAt() below when
    // a neighbor might legitimately fall outside the grid.
    float& U(int i, int j) { return u[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx + 1) + static_cast<std::size_t>(i)]; }
    float U(int i, int j) const { return u[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx + 1) + static_cast<std::size_t>(i)]; }
    float& V(int i, int j) { return v[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    float V(int i, int j) const { return v[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    float& Uw(int i, int j) { return uWeight[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx + 1) + static_cast<std::size_t>(i)]; }
    float& Vw(int i, int j) { return vWeight[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }

    CellType& cell(int i, int j) { return cellType[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    CellType cell(int i, int j) const { return cellType[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    float& P(int i, int j) { return pressure[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
    float P(int i, int j) const { return pressure[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }

    // Cell type for a possibly-out-of-range neighbor: treated as Solid,
    // which is exactly the boundary condition we want (the domain edge acts
    // like an infinitely thick wall, no different from the explicit
    // one-cell-thick Solid ring classifyCells() paints along i=0/nx-1/
    // j=0/ny-1).
    CellType cellTypeAt(int i, int j) const {
        if (i < 0 || i >= nx || j < 0 || j >= ny) return CellType::Solid;
        return cell(i, j);
    }
};

} // namespace fluids::flip
