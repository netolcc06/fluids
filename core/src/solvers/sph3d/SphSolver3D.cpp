#include "fluids/solvers/sph3d/SphSolver3D.h"

#include <algorithm>
#include <cmath>

#include "fluids/solvers/sph3d/Kernels3D.h"

namespace fluids::sph3d {

namespace {
// Tait equation-of-state exponent. Fixed (not exposed as a tunable param),
// same as the 2D solver — per Becker & Teschner 2007 / Monaghan's WCSPH
// formulation.
constexpr float kTaitGamma = 7.0f;
} // namespace

void SphSolver3D::seedRegion(float x, float y, float z, float width, float height, float depth, float vx,
                              float vy, float vz) {
    const float spacing = scenario_.spacing;
    if (spacing <= 0.0f || width <= 0.0f || height <= 0.0f || depth <= 0.0f) return;

    const int nx = std::max(1, static_cast<int>(width / spacing));
    const int ny = std::max(1, static_cast<int>(height / spacing));
    const int nz = std::max(1, static_cast<int>(depth / spacing));

    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                const float px = x + (static_cast<float>(ix) + 0.5f) * spacing;
                const float py = y + (static_cast<float>(iy) + 0.5f) * spacing;
                const float pz = z + (static_cast<float>(iz) + 0.5f) * spacing;
                positions_.emplace_back(px, py, pz);
                velocities_.emplace_back(vx, vy, vz);
            }
        }
    }
}

void SphSolver3D::reset(const Scenario3D& scenario) {
    scenario_ = scenario;

    positions_.clear();
    velocities_.clear();

    seedRegion(scenario.regionX, scenario.regionY, scenario.regionZ, scenario.regionWidth,
               scenario.regionHeight, scenario.regionDepth, scenario.velocityX, scenario.velocityY,
               scenario.velocityZ);
    if (scenario.hasRegion2) {
        seedRegion(scenario.region2X, scenario.region2Y, scenario.region2Z, scenario.region2Width,
                   scenario.region2Height, scenario.region2Depth, scenario.region2VelocityX,
                   scenario.region2VelocityY, scenario.region2VelocityZ);
    }

    const std::size_t n = positions_.size();
    accelerations_.assign(n, Vec3{0.0f, 0.0f, 0.0f});
    densities_.assign(n, 0.0f);
    pressures_.assign(n, 0.0f);
    neighborScratch_.clear();
}

void SphSolver3D::computeDensityPressure(const SphParams3D& params, const UniformGrid3D& grid) {
    const float h = params.smoothingRadius;
    const std::size_t n = positions_.size();

    for (std::size_t i = 0; i < n; ++i) {
        neighborScratch_.clear();
        grid.queryNeighbors(positions_[i], h, neighborScratch_);

        float density = 0.0f;
        for (int j : neighborScratch_) {
            const float r = (positions_[i] - positions_[static_cast<std::size_t>(j)]).length();
            density += params.particleMass * poly6(r, h);
        }
        densities_[i] = density;

        // Tait EOS. Negative pressure (density below rest) is clamped to
        // zero — same rationale as 2D: at a free surface a particle simply
        // has fewer neighbors, not "torn" fluid, and clamping avoids
        // spurious attraction/clumping there.
        const float ratio = density / params.restDensity;
        const float rawPressure = params.gasStiffness * (std::pow(ratio, kTaitGamma) - 1.0f);
        pressures_[i] = std::max(0.0f, rawPressure);
    }
}

void SphSolver3D::computeForces(const SphParams3D& params, const UniformGrid3D& grid) {
    const float h = params.smoothingRadius;
    const std::size_t n = positions_.size();

    for (std::size_t i = 0; i < n; ++i) {
        neighborScratch_.clear();
        grid.queryNeighbors(positions_[i], h, neighborScratch_);

        Vec3 pressureForce{0.0f, 0.0f, 0.0f};
        Vec3 viscosityForce{0.0f, 0.0f, 0.0f};

        const float rhoI = densities_[i];
        const float pI = pressures_[i];

        for (int jIdx : neighborScratch_) {
            const auto j = static_cast<std::size_t>(jIdx);
            if (j == i) continue;

            const Vec3 rij = positions_[i] - positions_[j];
            const float r = rij.length();
            if (r <= 0.0f || r > h) continue;

            const float rhoJ = densities_[j];
            if (rhoJ <= 0.0f) continue;

            // Symmetric (momentum-conserving) pressure force.
            const Vec3 grad = spikyGradient(rij, r, h);
            const float pressureTerm = pI / (rhoI * rhoI) + pressures_[j] / (rhoJ * rhoJ);
            pressureForce += grad * (-params.particleMass * pressureTerm);

            // Laplacian viscosity: diffuses relative velocity.
            const float lap = viscosityLaplacian(r, h);
            viscosityForce += (velocities_[j] - velocities_[i]) * (params.particleMass * lap / rhoJ);
        }
        viscosityForce *= params.viscosity;

        Vec3 accel{0.0f, params.gravity, 0.0f};
        if (rhoI > 0.0f) {
            accel += (pressureForce + viscosityForce) * (1.0f / rhoI);
        }
        accelerations_[i] = accel;
    }
}

void SphSolver3D::integrate(const SphParams3D& params, float dt) {
    // Symplectic (semi-implicit) Euler — same as 2D.
    const std::size_t n = positions_.size();
    for (std::size_t i = 0; i < n; ++i) {
        velocities_[i] += accelerations_[i] * dt;
        positions_[i] += velocities_[i] * dt;
    }
    (void)params;
}

void SphSolver3D::enforceBoundaries(const SphParams3D& params) {
    const float damping = params.boundaryDamping;
    const float width = scenario_.domainWidth;
    const float height = scenario_.domainHeight;
    const float depth = scenario_.domainDepth;

    for (std::size_t i = 0; i < positions_.size(); ++i) {
        Vec3& p = positions_[i];
        Vec3& v = velocities_[i];

        if (p.x < 0.0f) {
            p.x = 0.0f;
            if (v.x < 0.0f) v.x = -v.x * damping;
        } else if (p.x > width) {
            p.x = width;
            if (v.x > 0.0f) v.x = -v.x * damping;
        }

        if (p.y < 0.0f) {
            p.y = 0.0f;
            if (v.y < 0.0f) v.y = -v.y * damping;
        } else if (p.y > height) {
            p.y = height;
            if (v.y > 0.0f) v.y = -v.y * damping;
        }

        if (p.z < 0.0f) {
            p.z = 0.0f;
            if (v.z < 0.0f) v.z = -v.z * damping;
        } else if (p.z > depth) {
            p.z = depth;
            if (v.z > 0.0f) v.z = -v.z * damping;
        }
    }
}

void SphSolver3D::step(const SphParams3D& params, float dt) {
    if (positions_.empty()) return;

    // Defensive clamp: a runaway dt would otherwise blow the solver up
    // regardless of how carefully the frontend fixed-timesteps its
    // substeps. Same clamp value as the 2D solver.
    dt = std::clamp(dt, 0.0f, 1.0f / 30.0f);
    if (dt <= 0.0f) return;

    UniformGrid3D grid(std::max(params.smoothingRadius, 1e-4f));
    grid.build(positions_);

    computeDensityPressure(params, grid);
    computeForces(params, grid);
    integrate(params, dt);
    enforceBoundaries(params);
}

} // namespace fluids::sph3d
