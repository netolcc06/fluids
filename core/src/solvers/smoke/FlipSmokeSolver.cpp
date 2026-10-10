#include "fluids/solvers/smoke/FlipSmokeSolver.h"

#include <algorithm>
#include <cmath>

#include "fluids/common/Vec2.h"

namespace fluids::smoke {

using flip::CellType;
using flip::MacGrid;

namespace {

// Same fixed Jacobi iteration count as flip::FlipSolver's water pressure
// solve, and for the same reason (simplest thing that's correct and easy to
// verify — see FlipSolver.cpp's comment). Smoke's Poisson system is a
// slightly different shape (see projectPressure() below: every non-Solid
// cell is Fluid, there's no Air/Dirichlet boundary at all), but Jacobi
// convergence behavior for a grid in this size range is comparable, and 60
// iterations was likewise verified (see tests/core/test_flip_smoke_solver.cpp)
// to bring interior divergence well under 0.1 for the default cellSize.
constexpr int kPressureIterations = 60;

struct BiCoeffs {
    int i0, j0;
    float tx, ty;
};

// Identical clamped-bilinear-index scheme to flip::FlipSolver's
// bilinearCoeffs (duplicated here rather than shared — see class-level
// comment on why smoke doesn't force a shared abstraction with water for a
// ~30-line helper).
BiCoeffs bilinearCoeffs(float gx, float gy, int w, int h) {
    gx = std::clamp(gx, 0.0f, static_cast<float>(w - 1));
    gy = std::clamp(gy, 0.0f, static_cast<float>(h - 1));
    int i0 = std::clamp(static_cast<int>(gx), 0, w - 2);
    int j0 = std::clamp(static_cast<int>(gy), 0, h - 2);
    float tx = std::clamp(gx - static_cast<float>(i0), 0.0f, 1.0f);
    float ty = std::clamp(gy - static_cast<float>(j0), 0.0f, 1.0f);
    return {i0, j0, tx, ty};
}

float bilinearSample(const std::vector<float>& field, int w, int h, float gx, float gy) {
    const BiCoeffs c = bilinearCoeffs(gx, gy, w, h);
    const auto at = [&](int i, int j) { return field[static_cast<std::size_t>(j) * w + i]; };
    return (1.0f - c.tx) * (1.0f - c.ty) * at(c.i0, c.j0) + c.tx * (1.0f - c.ty) * at(c.i0 + 1, c.j0) +
           (1.0f - c.tx) * c.ty * at(c.i0, c.j0 + 1) + c.tx * c.ty * at(c.i0 + 1, c.j0 + 1);
}

// Grid-space coordinates for the u-face grid ((nx+1) x ny samples, face
// u(i,j) at world (i*h, (j+0.5)*h)), the v-face grid (nx x (ny+1) samples,
// face v(i,j) at world ((i+0.5)*h, j*h)), and cell-centered scalar fields
// (nx x ny samples, cell (i,j) center at world ((i+0.5)*h, (j+0.5)*h)).
// Same convention as FlipSolver's uCoords/vCoords, plus a cell-centered
// variant water's solver has no need for (particles never sample
// cell-centered fields there).
void uCoords(const Vec2& p, float h, float& gx, float& gy) {
    gx = p.x / h;
    gy = p.y / h - 0.5f;
}
void vCoords(const Vec2& p, float h, float& gx, float& gy) {
    gx = p.x / h - 0.5f;
    gy = p.y / h;
}
void cellCoords(const Vec2& p, float h, float& gx, float& gy) {
    gx = p.x / h - 0.5f;
    gy = p.y / h - 0.5f;
}

// Bilinear-samples the velocity field (uField/vField, laid out like
// grid_.u/grid_.v) at an arbitrary world position.
Vec2 sampleVelocity(const std::vector<float>& uField, const std::vector<float>& vField, const Vec2& pos, float h,
                     int nx, int ny) {
    float gx, gy;
    uCoords(pos, h, gx, gy);
    const float vx = bilinearSample(uField, nx + 1, ny, gx, gy);
    vCoords(pos, h, gx, gy);
    const float vy = bilinearSample(vField, nx, ny + 1, gx, gy);
    return Vec2(vx, vy);
}

float sampleCellField(const std::vector<float>& field, const Vec2& pos, float h, int nx, int ny) {
    float gx, gy;
    cellCoords(pos, h, gx, gy);
    return bilinearSample(field, nx, ny, gx, gy);
}

} // namespace

void FlipSmokeSolver::reset(const SmokeScenario& scenario, const FlipSmokeParams& params) {
    scenario_ = scenario;

    const float cellSize = std::max(params.cellSize, 1e-4f);
    // Same ceil-not-round + minimum-3 reasoning as FlipSolver::reset(): the
    // grid must cover the full domain, and the one-cell-thick Solid ring
    // needs at least one interior row/column left over.
    int nx = static_cast<int>(std::ceil(scenario.domainWidth / cellSize));
    int ny = static_cast<int>(std::ceil(scenario.domainHeight / cellSize));
    nx = std::max(nx, 3);
    ny = std::max(ny, 3);
    grid_.resize(nx, ny, cellSize);

    const std::size_t cellCount = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
    density_.assign(cellCount, 0.0f);
    // Seed the whole field at ambient rather than 0 so a caller using a
    // non-zero ambientTemperature doesn't see a one-step transient of the
    // dissipation term relaxing every cell from 0 up to ambient.
    temperature_.assign(cellCount, params.ambientTemperature);

    classifyCells();
}

void FlipSmokeSolver::classifyCells() {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    // Every cell starts Fluid ("air that can flow") — smoke has no
    // particle-occupancy notion of Air vs. Fluid the way water does; the
    // whole interior is always flow-carrying. Classification never changes
    // after this (no particles to move in/out of cells), so reset() is the
    // only place this runs.
    std::fill(grid_.cellType.begin(), grid_.cellType.end(), CellType::Fluid);
    for (int i = 0; i < nx; ++i) {
        grid_.cell(i, 0) = CellType::Solid;
        grid_.cell(i, ny - 1) = CellType::Solid;
    }
    for (int j = 0; j < ny; ++j) {
        grid_.cell(0, j) = CellType::Solid;
        grid_.cell(nx - 1, j) = CellType::Solid;
    }
}

void FlipSmokeSolver::emitSource(const FlipSmokeParams& params, float dt) {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    const float h = grid_.h;

    const float x0 = scenario_.sourceX;
    const float x1 = scenario_.sourceX + scenario_.sourceWidth;
    const float y0 = scenario_.sourceY;
    const float y1 = scenario_.sourceY + scenario_.sourceHeight;

    const float addAmount = scenario_.emissionRate * dt;
    const float tempBlend = std::clamp(scenario_.emissionRate * dt, 0.0f, 1.0f);

    for (int j = 0; j < ny; ++j) {
        const float cy = (static_cast<float>(j) + 0.5f) * h;
        if (cy < y0 || cy > y1) continue;
        for (int i = 0; i < nx; ++i) {
            const float cx = (static_cast<float>(i) + 0.5f) * h;
            if (cx < x0 || cx > x1) continue;
            if (grid_.cell(i, j) == CellType::Solid) continue;

            const std::size_t idx = static_cast<std::size_t>(j) * nx + i;
            density_[idx] = std::clamp(density_[idx] + addAmount, 0.0f, 1.0f);
            temperature_[idx] = temperature_[idx] + (params.sourceTemperature - temperature_[idx]) * tempBlend;
        }
    }
}

void FlipSmokeSolver::applyBuoyancy(const FlipSmokeParams& params, float dt) {
    const int nx = grid_.nx;
    const int ny = grid_.ny;

    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            // Average the two cell-centered temperatures adjacent to this
            // v-face, falling back to ambient for the face that sits on the
            // outer domain edge (j==0 has no cell below, j==ny has no cell
            // above) — these faces are boundary faces zeroed by
            // enforceSolidFaces() right after anyway, so the fallback value
            // only matters in that it stays finite and doesn't bias any
            // neighbor.
            const float tBelow = (j > 0) ? temperature_[static_cast<std::size_t>(j - 1) * nx + i] : params.ambientTemperature;
            const float tAbove = (j < ny) ? temperature_[static_cast<std::size_t>(j) * nx + i] : params.ambientTemperature;
            const float tFace = 0.5f * (tBelow + tAbove);
            grid_.V(i, j) += dt * params.buoyancy * (tFace - params.ambientTemperature);
        }
    }
}

void FlipSmokeSolver::enforceSolidFaces() {
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

void FlipSmokeSolver::projectPressure() {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    const float h = grid_.h;

    // Divergence per non-Solid cell (== every cell that isn't part of the
    // boundary ring, since smoke has no Air state — see classifyCells()).
    std::vector<float> div(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0.0f);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (grid_.cell(i, j) != CellType::Fluid) continue;
            div[static_cast<std::size_t>(j) * nx + i] =
                (grid_.U(i + 1, j) - grid_.U(i, j) + grid_.V(i, j + 1) - grid_.V(i, j)) / h;
        }
    }

    // Simplified version of FlipSolver's variable-degree Jacobi Poisson
    // solve: because there is no Air cell type for smoke, every non-Solid
    // neighbor of a Fluid cell is itself Fluid, so the "count non-Solid
    // neighbors for diag, but only sum pressure from Fluid neighbors" split
    // FlipSolver needs collapses into one thing (a neighbor either
    // contributes to both diag and the sum, or it's Solid and contributes
    // to neither). The resulting system has no Dirichlet boundary anywhere
    // (Air normally supplies an implicit q=0 condition; here the domain is
    // entirely walled in by the Solid ring) — a pure Neumann problem, same
    // as the textbook "smoke in a closed box" formulation. That system is
    // solvable only up to an additive constant, but Jacobi starting from an
    // all-zero initial guess converges to one particular consistent
    // solution, and only pressure *differences* (used below to correct face
    // velocities) ever matter — the undetermined constant cancels exactly.
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

                if (grid_.cellTypeAt(i - 1, j) != CellType::Solid) {
                    diag += 1.0f;
                    sumN += grid_.pressure[idx - 1];
                }
                if (grid_.cellTypeAt(i + 1, j) != CellType::Solid) {
                    diag += 1.0f;
                    sumN += grid_.pressure[idx + 1];
                }
                if (grid_.cellTypeAt(i, j - 1) != CellType::Solid) {
                    diag += 1.0f;
                    sumN += grid_.pressure[idx - static_cast<std::size_t>(nx)];
                }
                if (grid_.cellTypeAt(i, j + 1) != CellType::Solid) {
                    diag += 1.0f;
                    sumN += grid_.pressure[idx + static_cast<std::size_t>(nx)];
                }

                qNext[idx] = (diag > 0.0f) ? (sumN - h * div[idx]) / diag : 0.0f;
            }
        }
        std::swap(grid_.pressure, qNext);
    }

    for (int j = 0; j < ny; ++j) {
        for (int i = 1; i < nx; ++i) {
            const CellType left = grid_.cell(i - 1, j);
            const CellType right = grid_.cell(i, j);
            if (left == CellType::Solid || right == CellType::Solid) continue;
            const float qL = grid_.pressure[static_cast<std::size_t>(j) * nx + (i - 1)];
            const float qR = grid_.pressure[static_cast<std::size_t>(j) * nx + i];
            grid_.U(i, j) -= (qR - qL);
        }
    }
    for (int j = 1; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const CellType bottom = grid_.cell(i, j - 1);
            const CellType top = grid_.cell(i, j);
            if (bottom == CellType::Solid || top == CellType::Solid) continue;
            const float qB = grid_.pressure[static_cast<std::size_t>(j - 1) * nx + i];
            const float qT = grid_.pressure[static_cast<std::size_t>(j) * nx + i];
            grid_.V(i, j) -= (qT - qB);
        }
    }
}

void FlipSmokeSolver::advect(float dt) {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    const float h = grid_.h;

    // Single snapshot of the post-projection, divergence-free velocity
    // field, used to trace backward for BOTH velocity self-advection and
    // density/temperature advection (see FlipSmokeSolver.h's pipeline
    // comment on step 4/5 — both read from this one "current" field rather
    // than density/temperature advection using velocity that's already
    // been self-advected this step, which would make the two passes order-
    // dependent for no benefit).
    const std::vector<float> uSrc = grid_.u;
    const std::vector<float> vSrc = grid_.v;
    const std::vector<float> densitySrc = density_;
    const std::vector<float> temperatureSrc = temperature_;

    std::vector<float> uNew(grid_.u.size(), 0.0f);
    std::vector<float> vNew(grid_.v.size(), 0.0f);
    std::vector<float> densityNew(density_.size(), 0.0f);
    std::vector<float> temperatureNew(temperature_.size(), 0.0f);

    // Self-advect u faces.
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i <= nx; ++i) {
            const Vec2 facePos(static_cast<float>(i) * h, (static_cast<float>(j) + 0.5f) * h);
            const Vec2 vel = sampleVelocity(uSrc, vSrc, facePos, h, nx, ny);
            const Vec2 backPos = facePos - vel * dt;
            float gx, gy;
            uCoords(backPos, h, gx, gy);
            uNew[static_cast<std::size_t>(j) * (nx + 1) + i] = bilinearSample(uSrc, nx + 1, ny, gx, gy);
        }
    }
    // Self-advect v faces.
    for (int j = 0; j <= ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Vec2 facePos((static_cast<float>(i) + 0.5f) * h, static_cast<float>(j) * h);
            const Vec2 vel = sampleVelocity(uSrc, vSrc, facePos, h, nx, ny);
            const Vec2 backPos = facePos - vel * dt;
            float gx, gy;
            vCoords(backPos, h, gx, gy);
            vNew[static_cast<std::size_t>(j) * nx + i] = bilinearSample(vSrc, nx, ny + 1, gx, gy);
        }
    }
    // Advect density/temperature, tracing through the SAME uSrc/vSrc
    // snapshot (not uNew/vNew).
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const std::size_t idx = static_cast<std::size_t>(j) * nx + i;
            if (grid_.cell(i, j) == CellType::Solid) {
                densityNew[idx] = 0.0f;
                temperatureNew[idx] = temperatureSrc[idx];
                continue;
            }
            const Vec2 cellPos((static_cast<float>(i) + 0.5f) * h, (static_cast<float>(j) + 0.5f) * h);
            const Vec2 vel = sampleVelocity(uSrc, vSrc, cellPos, h, nx, ny);
            const Vec2 backPos = cellPos - vel * dt;
            densityNew[idx] = sampleCellField(densitySrc, backPos, h, nx, ny);
            temperatureNew[idx] = sampleCellField(temperatureSrc, backPos, h, nx, ny);
        }
    }

    // grid_.u/v are never exposed to JS (only density/temperature are — see
    // FlipSmokeSimulation), so replacing them wholesale via move is fine.
    // density_/temperature_ ARE exposed via getDensityFieldPtr()/
    // getTemperatureFieldPtr(), and that contract promises a pointer stable
    // across step() calls (same as FlipSolver's positions_/velocities_,
    // which are only ever mutated in place, never reassigned, during
    // step()) — so these two are copied element-by-element into the
    // EXISTING buffers instead of being move-assigned from a new vector,
    // which would silently swap in a different heap allocation (and did,
    // before this was caught empirically — see docs/flip-smoke-sim-api.md's
    // pointer-stability section).
    grid_.u = std::move(uNew);
    grid_.v = std::move(vNew);
    std::copy(densityNew.begin(), densityNew.end(), density_.begin());
    std::copy(temperatureNew.begin(), temperatureNew.end(), temperature_.begin());
}

void FlipSmokeSolver::applyDissipation(const FlipSmokeParams& params, float dt) {
    const int nx = grid_.nx;
    const int ny = grid_.ny;
    const float densityDecay = std::clamp(params.densityDissipation * dt, 0.0f, 1.0f);
    const float tempRelax = std::clamp(params.temperatureDissipation * dt, 0.0f, 1.0f);

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (grid_.cell(i, j) == CellType::Solid) continue;
            const std::size_t idx = static_cast<std::size_t>(j) * nx + i;
            density_[idx] = std::max(0.0f, density_[idx] * (1.0f - densityDecay));
            temperature_[idx] += (params.ambientTemperature - temperature_[idx]) * tempRelax;
        }
    }
}

void FlipSmokeSolver::step(const FlipSmokeParams& params, float dt) {
    // Same defensive dt clamp as FlipSolver::step / SPHSolver::step.
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    emitSource(params, dt);
    applyBuoyancy(params, dt);
    enforceSolidFaces();
    projectPressure();
    enforceSolidFaces();

    advect(dt);

    applyDissipation(params, dt);
    enforceSolidFaces();
}

} // namespace fluids::smoke
