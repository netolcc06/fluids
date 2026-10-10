#include "fluids/solvers/flip/FlipSolver.h"

#include <algorithm>
#include <cmath>

namespace fluids::flip {

namespace {

// Fixed Jacobi iteration count for the pressure Poisson solve. Jacobi
// (rather than a proper preconditioned CG) is deliberately the simplest
// thing that can be correct and easy to debug/verify for Phase 2 — per the
// master plan's explicit "correctness over performance" call for this
// phase. 60 iterations is comfortably inside the 40-80 range that
// converges well enough (near-zero divergence, see
// tests/core/test_flip_solver.cpp) for grids in the ~50-100 cells/side
// range the default cellSize produces.
constexpr int kPressureIterations = 60;

// Velocity extrapolation band depth, in cells. 2-3 is the standard
// recommendation (enough that a particle sitting right at the free surface
// never samples a stale/never-written Air-cell value during G2P bilinear
// interpolation, which reaches one cell beyond the particle's own cell).
constexpr int kExtrapolationDepth = 3;

constexpr float kMinWeight = 1e-8f;

struct BiCoeffs {
    int i0, j0;
    float tx, ty;
};

// Clamps continuous grid-space coordinates (gx, gy) into a field of size
// w x h and returns the lower-left sample index plus fractional weights,
// such that (i0, j0), (i0+1, j0), (i0, j0+1), (i0+1, j0+1) are all valid
// indices. Used identically by P2G (scatter) and G2P (gather) so the two
// are exact inverses of each other on a constant field.
BiCoeffs bilinearCoeffs(float gx, float gy, int w, int h) {
    gx = std::clamp(gx, 0.0f, static_cast<float>(w - 1));
    gy = std::clamp(gy, 0.0f, static_cast<float>(h - 1));
    int i0 = std::clamp(static_cast<int>(gx), 0, w - 2);
    int j0 = std::clamp(static_cast<int>(gy), 0, h - 2);
    float tx = std::clamp(gx - static_cast<float>(i0), 0.0f, 1.0f);
    float ty = std::clamp(gy - static_cast<float>(j0), 0.0f, 1.0f);
    return {i0, j0, tx, ty};
}

// Grid-space coordinates of a world position on the u-face grid ((nx+1) x
// ny samples, face u(i,j) at x=i*h, y=(j+0.5)*h) and the v-face grid (nx x
// (ny+1) samples, face v(i,j) at x=(i+0.5)*h, y=j*h).
void uCoords(const Vec2& p, float h, float& gx, float& gy) {
    gx = p.x / h;
    gy = p.y / h - 0.5f;
}
void vCoords(const Vec2& p, float h, float& gx, float& gy) {
    gx = p.x / h - 0.5f;
    gy = p.y / h;
}

float bilinearSample(const std::vector<float>& field, int w, int h, float gx, float gy) {
    const BiCoeffs c = bilinearCoeffs(gx, gy, w, h);
    const auto at = [&](int i, int j) { return field[static_cast<std::size_t>(j) * w + i]; };
    return (1.0f - c.tx) * (1.0f - c.ty) * at(c.i0, c.j0) + c.tx * (1.0f - c.ty) * at(c.i0 + 1, c.j0) +
           (1.0f - c.tx) * c.ty * at(c.i0, c.j0 + 1) + c.tx * c.ty * at(c.i0 + 1, c.j0 + 1);
}

} // namespace

void FlipSolver::seedRegion(float x, float y, float width, float height, float vx, float vy) {
    const float spacing = scenario_.spacing;
    if (spacing <= 0.0f || width <= 0.0f || height <= 0.0f) return;

    const int nx = std::max(1, static_cast<int>(width / spacing));
    const int ny = std::max(1, static_cast<int>(height / spacing));

    for (int iy = 0; iy < ny; ++iy) {
        for (int ix = 0; ix < nx; ++ix) {
            const float px = x + (static_cast<float>(ix) + 0.5f) * spacing;
            const float py = y + (static_cast<float>(iy) + 0.5f) * spacing;
            positions_.emplace_back(px, py);
            velocities_.emplace_back(vx, vy);
        }
    }
}

void FlipSolver::reset(const Scenario& scenario, const FlipParams& params) {
    scenario_ = scenario;

    positions_.clear();
    velocities_.clear();

    seedRegion(scenario.regionX, scenario.regionY, scenario.regionWidth, scenario.regionHeight,
               scenario.velocityX, scenario.velocityY);
    if (scenario.hasRegion2) {
        seedRegion(scenario.region2X, scenario.region2Y, scenario.region2Width, scenario.region2Height,
                   scenario.region2VelocityX, scenario.region2VelocityY);
    }

    const float cellSize = std::max(params.cellSize, 1e-4f);
    // ceil (not round) so the grid always covers at least the full domain —
    // otherwise particles at/near the far edge could sit outside the last
    // grid cell.
    int nx = static_cast<int>(std::ceil(scenario.domainWidth / cellSize));
    int ny = static_cast<int>(std::ceil(scenario.domainHeight / cellSize));
    // Minimum 3 so the one-cell-thick Solid boundary ring (see
    // classifyCells) always leaves at least one interior column/row.
    nx = std::max(nx, 3);
    ny = std::max(ny, 3);
    grid_.resize(nx, ny, cellSize);

    uOld_.assign(grid_.u.size(), 0.0f);
    vOld_.assign(grid_.v.size(), 0.0f);
    uValid_.assign(grid_.u.size(), 0);
    vValid_.assign(grid_.v.size(), 0);
}

void FlipSolver::particleToGrid() {
    grid_.clearVelocities();
    const float h = grid_.h;
    const int uw = grid_.nx + 1;
    const int uh = grid_.ny;
    const int vw = grid_.nx;
    const int vh = grid_.ny + 1;

    for (std::size_t p = 0; p < positions_.size(); ++p) {
        const Vec2& pos = positions_[p];
        const Vec2& vel = velocities_[p];

        float gx, gy;
        uCoords(pos, h, gx, gy);
        BiCoeffs c = bilinearCoeffs(gx, gy, uw, uh);
        float w00 = (1.0f - c.tx) * (1.0f - c.ty);
        float w10 = c.tx * (1.0f - c.ty);
        float w01 = (1.0f - c.tx) * c.ty;
        float w11 = c.tx * c.ty;
        grid_.U(c.i0, c.j0) += vel.x * w00;
        grid_.Uw(c.i0, c.j0) += w00;
        grid_.U(c.i0 + 1, c.j0) += vel.x * w10;
        grid_.Uw(c.i0 + 1, c.j0) += w10;
        grid_.U(c.i0, c.j0 + 1) += vel.x * w01;
        grid_.Uw(c.i0, c.j0 + 1) += w01;
        grid_.U(c.i0 + 1, c.j0 + 1) += vel.x * w11;
        grid_.Uw(c.i0 + 1, c.j0 + 1) += w11;

        vCoords(pos, h, gx, gy);
        c = bilinearCoeffs(gx, gy, vw, vh);
        w00 = (1.0f - c.tx) * (1.0f - c.ty);
        w10 = c.tx * (1.0f - c.ty);
        w01 = (1.0f - c.tx) * c.ty;
        w11 = c.tx * c.ty;
        grid_.V(c.i0, c.j0) += vel.y * w00;
        grid_.Vw(c.i0, c.j0) += w00;
        grid_.V(c.i0 + 1, c.j0) += vel.y * w10;
        grid_.Vw(c.i0 + 1, c.j0) += w10;
        grid_.V(c.i0, c.j0 + 1) += vel.y * w01;
        grid_.Vw(c.i0, c.j0 + 1) += w01;
        grid_.V(c.i0 + 1, c.j0 + 1) += vel.y * w11;
        grid_.Vw(c.i0 + 1, c.j0 + 1) += w11;
    }

    for (std::size_t k = 0; k < grid_.u.size(); ++k) {
        if (grid_.uWeight[k] > kMinWeight) {
            grid_.u[k] /= grid_.uWeight[k];
            uValid_[k] = 1;
        } else {
            grid_.u[k] = 0.0f;
            uValid_[k] = 0;
        }
    }
    for (std::size_t k = 0; k < grid_.v.size(); ++k) {
        if (grid_.vWeight[k] > kMinWeight) {
            grid_.v[k] /= grid_.vWeight[k];
            vValid_[k] = 1;
        } else {
            grid_.v[k] = 0.0f;
            vValid_[k] = 0;
        }
    }
}

void FlipSolver::classifyCells() {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    std::fill(grid_.cellType.begin(), grid_.cellType.end(), CellType::Air);

    // One-cell-thick Solid boundary ring: the domain has no interior
    // obstacles yet (per the master plan), so the only Solid cells are the
    // outer border. This insets the *effective* fluid-carrying region by
    // one cellSize on each side relative to the nominal
    // [0,domainWidth]x[0,domainHeight] box — negligible at the default
    // cellSize (~1-2% of domain width) but worth knowing if cellSize is set
    // large relative to the domain (see docs/flip-sim-api.md).
    for (int i = 0; i < nx; ++i) {
        grid_.cell(i, 0) = CellType::Solid;
        grid_.cell(i, ny - 1) = CellType::Solid;
    }
    for (int j = 0; j < ny; ++j) {
        grid_.cell(0, j) = CellType::Solid;
        grid_.cell(nx - 1, j) = CellType::Solid;
    }

    const float h = grid_.h;
    for (const Vec2& pos : positions_) {
        const int ci = std::clamp(static_cast<int>(std::floor(pos.x / h)), 0, nx - 1);
        const int cj = std::clamp(static_cast<int>(std::floor(pos.y / h)), 0, ny - 1);
        if (grid_.cell(ci, cj) != CellType::Solid) {
            grid_.cell(ci, cj) = CellType::Fluid;
        }
    }
}

void FlipSolver::applyGravity(float gravity, float dt) {
    for (float& val : grid_.v) val += gravity * dt;
}

void FlipSolver::enforceSolidFaces() {
    const int nx = grid_.nx;
    const int ny = grid_.ny;

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i <= nx; ++i) {
            const CellType left = grid_.cellTypeAt(i - 1, j);
            const CellType right = grid_.cellTypeAt(i, j);
            if (left == CellType::Solid || right == CellType::Solid) {
                grid_.U(i, j) = 0.0f;
            }
        }
    }
    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const CellType bottom = grid_.cellTypeAt(i, j - 1);
            const CellType top = grid_.cellTypeAt(i, j);
            if (bottom == CellType::Solid || top == CellType::Solid) {
                grid_.V(i, j) = 0.0f;
            }
        }
    }
}

void FlipSolver::projectPressure() {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    const float h = grid_.h;

    // Divergence per fluid cell, using current (already solid-zeroed)
    // face velocities so solid walls never appear as a source/sink.
    std::vector<float> div(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0.0f);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (grid_.cell(i, j) != CellType::Fluid) continue;
            div[static_cast<std::size_t>(j) * nx + i] =
                (grid_.U(i + 1, j) - grid_.U(i, j) + grid_.V(i, j + 1) - grid_.V(i, j)) / h;
        }
    }

    // Scale-free pressure variable q (= dt*p/(rho*h) in physical units, but
    // since rho/dt/h all cancel out of this Phase 2 formulation we solve
    // directly for the quantity subtracted from face velocities). Standard
    // variable-degree Poisson system (Bridson, "Fluid Simulation for
    // Computer Graphics", ch. 5): for fluid cell (i,j), diag = number of
    // non-Solid neighbors, off-diagonal contributions only from Fluid
    // neighbors (Air neighbors are an implicit q=0 Dirichlet boundary, so
    // they count toward diag but contribute nothing to the neighbor sum).
    std::fill(grid_.pressure.begin(), grid_.pressure.end(), 0.0f);
    std::vector<float> qNext(grid_.pressure.size(), 0.0f);

    for (int iter = 0; iter < kPressureIterations; ++iter) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                const std::size_t idx = static_cast<std::size_t>(j) * nx + i;
                if (grid_.cell(i, j) != CellType::Fluid) {
                    qNext[idx] = 0.0f;
                    continue;
                }
                float diag = 0.0f;
                float sumN = 0.0f;
                CellType t;

                t = grid_.cellTypeAt(i - 1, j);
                if (t != CellType::Solid) {
                    diag += 1.0f;
                    if (t == CellType::Fluid) sumN += grid_.pressure[static_cast<std::size_t>(j) * nx + (i - 1)];
                }
                t = grid_.cellTypeAt(i + 1, j);
                if (t != CellType::Solid) {
                    diag += 1.0f;
                    if (t == CellType::Fluid) sumN += grid_.pressure[static_cast<std::size_t>(j) * nx + (i + 1)];
                }
                t = grid_.cellTypeAt(i, j - 1);
                if (t != CellType::Solid) {
                    diag += 1.0f;
                    if (t == CellType::Fluid) sumN += grid_.pressure[static_cast<std::size_t>(j - 1) * nx + i];
                }
                t = grid_.cellTypeAt(i, j + 1);
                if (t != CellType::Solid) {
                    diag += 1.0f;
                    if (t == CellType::Fluid) sumN += grid_.pressure[static_cast<std::size_t>(j + 1) * nx + i];
                }

                qNext[idx] = (diag > 0.0f) ? (sumN - h * div[idx]) / diag : 0.0f;
            }
        }
        std::swap(grid_.pressure, qNext);
    }

    // Subtract the pressure gradient from every face between two non-Solid
    // cells where at least one side is Fluid. Air-Air faces are skipped
    // (both q=0, a no-op anyway); Solid-adjacent faces are left alone here
    // and explicitly re-zeroed by the caller's second enforceSolidFaces()
    // call, per the master plan's emphasis on not skipping that step.
    for (int j = 0; j < ny; ++j) {
        for (int i = 1; i < nx; ++i) {
            const CellType left = grid_.cell(i - 1, j);
            const CellType right = grid_.cell(i, j);
            if (left == CellType::Solid || right == CellType::Solid) continue;
            if (left != CellType::Fluid && right != CellType::Fluid) continue;
            const float qL = (left == CellType::Fluid) ? grid_.pressure[static_cast<std::size_t>(j) * nx + (i - 1)] : 0.0f;
            const float qR = (right == CellType::Fluid) ? grid_.pressure[static_cast<std::size_t>(j) * nx + i] : 0.0f;
            grid_.U(i, j) -= (qR - qL);
        }
    }
    for (int j = 1; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const CellType bottom = grid_.cell(i, j - 1);
            const CellType top = grid_.cell(i, j);
            if (bottom == CellType::Solid || top == CellType::Solid) continue;
            if (bottom != CellType::Fluid && top != CellType::Fluid) continue;
            const float qB = (bottom == CellType::Fluid) ? grid_.pressure[static_cast<std::size_t>(j - 1) * nx + i] : 0.0f;
            const float qT = (top == CellType::Fluid) ? grid_.pressure[static_cast<std::size_t>(j) * nx + i] : 0.0f;
            grid_.V(i, j) -= (qT - qB);
        }
    }
}

void FlipSolver::extrapolate(std::vector<float>& field, const std::vector<std::uint8_t>& validMask, int w, int h,
                              int depth) {
    std::vector<std::uint8_t> valid = validMask;
    for (int iter = 0; iter < depth; ++iter) {
        std::vector<std::uint8_t> nextValid = valid;
        for (int j = 0; j < h; ++j) {
            for (int i = 0; i < w; ++i) {
                const std::size_t idx = static_cast<std::size_t>(j) * w + i;
                if (valid[idx]) continue;
                float sum = 0.0f;
                int cnt = 0;
                if (i > 0 && valid[idx - 1]) { sum += field[idx - 1]; ++cnt; }
                if (i < w - 1 && valid[idx + 1]) { sum += field[idx + 1]; ++cnt; }
                if (j > 0 && valid[idx - static_cast<std::size_t>(w)]) { sum += field[idx - static_cast<std::size_t>(w)]; ++cnt; }
                if (j < h - 1 && valid[idx + static_cast<std::size_t>(w)]) { sum += field[idx + static_cast<std::size_t>(w)]; ++cnt; }
                if (cnt > 0) {
                    field[idx] = sum / static_cast<float>(cnt);
                    nextValid[idx] = 1;
                }
            }
        }
        valid = std::move(nextValid);
    }
}

void FlipSolver::gridToParticles(float flipRatio) {
    const float h = grid_.h;
    const int uw = grid_.nx + 1;
    const int uh = grid_.ny;
    const int vw = grid_.nx;
    const int vh = grid_.ny + 1;

    // Extrapolate BOTH the post-projection field and the pre-projection
    // (post-P2G) snapshot, using the identical validity mask captured right
    // after P2G. This is a deliberate refinement beyond the minimum: if
    // only the new field were extrapolated, a particle sampling a cell that
    // had no particle data at P2G time this step (uOld/vOld == 0 there,
    // never extrapolated) would see "change since P2G" spuriously equal to
    // the entire extrapolated value rather than the actual physical change,
    // spiking the FLIP contribution right at the newly-wetted free surface
    // — exactly the kind of surface artifact the master plan calls out.
    std::vector<float> uNew = grid_.u;
    std::vector<float> vNew = grid_.v;
    extrapolate(uNew, uValid_, uw, uh, kExtrapolationDepth);
    extrapolate(vNew, vValid_, vw, vh, kExtrapolationDepth);

    std::vector<float> uOldExtrap = uOld_;
    std::vector<float> vOldExtrap = vOld_;
    extrapolate(uOldExtrap, uValid_, uw, uh, kExtrapolationDepth);
    extrapolate(vOldExtrap, vValid_, vw, vh, kExtrapolationDepth);

    for (std::size_t p = 0; p < positions_.size(); ++p) {
        const Vec2& pos = positions_[p];
        float gx, gy;

        uCoords(pos, h, gx, gy);
        const float picX = bilinearSample(uNew, uw, uh, gx, gy);
        const float oldX = bilinearSample(uOldExtrap, uw, uh, gx, gy);

        vCoords(pos, h, gx, gy);
        const float picY = bilinearSample(vNew, vw, vh, gx, gy);
        const float oldY = bilinearSample(vOldExtrap, vw, vh, gx, gy);

        const float flipX = velocities_[p].x + (picX - oldX);
        const float flipY = velocities_[p].y + (picY - oldY);

        velocities_[p].x = flipRatio * flipX + (1.0f - flipRatio) * picX;
        velocities_[p].y = flipRatio * flipY + (1.0f - flipRatio) * picY;
    }
}

void FlipSolver::advect(float dt) {
    for (std::size_t i = 0; i < positions_.size(); ++i) {
        positions_[i] += velocities_[i] * dt;
    }
}

void FlipSolver::clampToDomain() {
    const float w = scenario_.domainWidth;
    const float hgt = scenario_.domainHeight;
    for (std::size_t i = 0; i < positions_.size(); ++i) {
        Vec2& p = positions_[i];
        Vec2& v = velocities_[i];

        if (p.x < 0.0f) {
            p.x = 0.0f;
            if (v.x < 0.0f) v.x = 0.0f;
        } else if (p.x > w) {
            p.x = w;
            if (v.x > 0.0f) v.x = 0.0f;
        }
        if (p.y < 0.0f) {
            p.y = 0.0f;
            if (v.y < 0.0f) v.y = 0.0f;
        } else if (p.y > hgt) {
            p.y = hgt;
            if (v.y > 0.0f) v.y = 0.0f;
        }
    }
}

void FlipSolver::step(const FlipParams& params, float dt) {
    if (positions_.empty()) return;

    // Same defensive dt clamp as SPHSolver::step: guards against a runaway
    // caller dt (e.g. a stalled tab resuming after a long pause).
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    particleToGrid();
    // Snapshot right after P2G, before gravity/projection touch the grid —
    // this is the "old" state the FLIP delta is measured against.
    uOld_ = grid_.u;
    vOld_ = grid_.v;

    classifyCells();
    applyGravity(params.gravity, dt);
    enforceSolidFaces();
    projectPressure();
    enforceSolidFaces();

    gridToParticles(std::clamp(params.flipRatio, 0.0f, 1.0f));
    advect(dt);
    clampToDomain();
}

} // namespace fluids::flip
