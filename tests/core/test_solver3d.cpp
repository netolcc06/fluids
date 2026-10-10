#include <algorithm>
#include <cmath>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "fluids/SphSimulation3D.h"
#include "fluids/common/UniformGrid3D.h"
#include "fluids/common/Vec3.h"
#include "fluids/scenario/Scenario3D.h"
#include "fluids/solvers/sph3d/Kernels3D.h"
#include "fluids/solvers/sph3d/SphParams3D.h"

using fluids::Scenario3D;
using fluids::SphSimulation3D;
using fluids::UniformGrid3D;
using fluids::Vec3;

namespace {

bool allFinite(const float* buf, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(buf[i])) return false;
    }
    return true;
}

// Buffers are [x,y,z] per particle (3 floats), NOT [x,y] (2 floats) like
// every 2D solver — see docs/sph3d-sim-api.md. Generic indexer below works
// for either the position or the velocity buffer, since both use the same
// 3-floats-per-particle layout, instead of hand-rolling `i*3+k` inline.
float component(const float* buf, std::size_t i, int comp) { return buf[i * 3 + static_cast<std::size_t>(comp)]; }

float averageHeight(const float* positions, std::size_t particleCount) {
    if (particleCount == 0) return 0.0f;
    double sum = 0.0;
    for (std::size_t i = 0; i < particleCount; ++i) {
        sum += component(positions, i, 1); // y component
    }
    return static_cast<float>(sum / static_cast<double>(particleCount));
}

float rmsSpeed(const float* vel, std::size_t n) {
    if (n == 0) return 0.0f;
    double sumSq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        float vx = component(vel, i, 0);
        float vy = component(vel, i, 1);
        float vz = component(vel, i, 2);
        sumSq += static_cast<double>(vx) * vx + static_cast<double>(vy) * vy + static_cast<double>(vz) * vz;
    }
    return static_cast<float>(std::sqrt(sumSq / static_cast<double>(n)));
}

} // namespace

TEST_CASE("3D particleMass calibration: rest-packed grid measures density close to restDensity",
          "[solver3d][calibration]") {
    // Verifies the particleMass ~= restDensity * spacing^3 rule of thumb
    // (SphParams3D.h) empirically: seed a rest-packed 3D lattice at the
    // default spacing, larger than one smoothing radius in every direction
    // so an interior particle has a full neighborhood, and check that its
    // Poly6 density estimate lands close to restDensity — the same kind of
    // check the 2D project's docs call for (a freshly-seeded rest-packed
    // grid should measure a density estimate close to restDensity).
    fluids::sph3d::SphParams3D params; // defaults
    const float spacing = fluids::makeDamBreak3DScenario().spacing;
    const float h = params.smoothingRadius;

    // Build a cube of particles wide enough that the very center one sees a
    // full spherical neighborhood within h.
    const int half = static_cast<int>(std::ceil(h / spacing)) + 2;
    std::vector<Vec3> points;
    for (int iz = -half; iz <= half; ++iz) {
        for (int iy = -half; iy <= half; ++iy) {
            for (int ix = -half; ix <= half; ++ix) {
                points.emplace_back(static_cast<float>(ix) * spacing, static_cast<float>(iy) * spacing,
                                     static_cast<float>(iz) * spacing);
            }
        }
    }

    UniformGrid3D grid(h);
    grid.build(points);

    const Vec3 center{0.0f, 0.0f, 0.0f};
    std::vector<int> neighbors;
    grid.queryNeighbors(center, h, neighbors);

    float density = 0.0f;
    for (int j : neighbors) {
        const float r = (points[static_cast<std::size_t>(j)] - center).length();
        density += params.particleMass * fluids::sph3d::poly6(r, h);
    }

    INFO("measured density = " << density << ", restDensity = " << params.restDensity
                                << ", neighbor count = " << neighbors.size());
    // particleMass's default (SphParams3D.h) was empirically corrected
    // (~1.468x the naive restDensity*spacing^3 figure) specifically so this
    // lattice sum lands within ~1% of restDensity — see the long comment
    // above SphParams3D::particleMass for the full derivation. A tight 10%
    // band here (tighter than a from-scratch tolerance would need) is a
    // deliberate regression guard: if this default or the kernel constants
    // ever drift, this test should catch a real miscalibration, not just a
    // wildly-wrong one.
    REQUIRE(density > params.restDensity * 0.9f);
    REQUIRE(density < params.restDensity * 1.1f);
}

TEST_CASE("SphSimulation3D default construction seeds particles", "[solver3d][smoke]") {
    SphSimulation3D sim;
    REQUIRE(sim.getParticleCount() > 0);
}

TEST_CASE("3D solver stability smoke test: no NaN/Inf, stays within domain bounds", "[solver3d][smoke]") {
    SphSimulation3D sim;
    Scenario3D scenario = fluids::makeDamBreak3DScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float dt = 1.0f / 240.0f;
    const int steps = 240 * 3; // 3 seconds
    const float margin = scenario.spacing; // small slack for boundary overshoot within a substep

    for (int i = 0; i < steps; ++i) {
        sim.step(dt);

        const float* pos = sim.getPositionsPtr();
        const float* vel = sim.getVelocitiesPtr();
        REQUIRE(allFinite(pos, n * 3));
        REQUIRE(allFinite(vel, n * 3));

        for (std::size_t p = 0; p < n; ++p) {
            float x = component(pos, p, 0);
            float y = component(pos, p, 1);
            float z = component(pos, p, 2);
            REQUIRE(x >= -margin);
            REQUIRE(x <= scenario.domainWidth + margin);
            REQUIRE(y >= -margin);
            REQUIRE(y <= scenario.domainHeight + margin);
            REQUIRE(z >= -margin);
            REQUIRE(z <= scenario.domainDepth + margin);
        }
    }
}

TEST_CASE("3D dam break scenario collapses and settles under gravity", "[solver3d][dambreak]") {
    SphSimulation3D sim;
    Scenario3D scenario = fluids::makeDamBreak3DScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float initialAvgHeight = averageHeight(sim.getPositionsPtr(), n);
    const auto spreadExtent = [&](const float* pos) {
        float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
        for (std::size_t i = 0; i < n; ++i) {
            minX = std::min(minX, component(pos, i, 0));
            maxX = std::max(maxX, component(pos, i, 0));
            minZ = std::min(minZ, component(pos, i, 2));
            maxZ = std::max(maxZ, component(pos, i, 2));
        }
        return std::make_pair(maxX - minX, maxZ - minZ);
    };
    const auto [initialSpreadX, initialSpreadZ] = spreadExtent(sim.getPositionsPtr());

    const float dt = 1.0f / 240.0f;
    const int steps = 240 * 3; // 3 seconds of simulated time
    for (int i = 0; i < steps; ++i) {
        sim.step(dt);
    }

    const float* pos = sim.getPositionsPtr();
    const float* vel = sim.getVelocitiesPtr();
    REQUIRE(allFinite(pos, n * 3));
    REQUIRE(allFinite(vel, n * 3));

    const float finalAvgHeight = averageHeight(pos, n);
    const auto [finalSpreadX, finalSpreadZ] = spreadExtent(pos);

    // The column collapses: average particle height drops...
    REQUIRE(finalAvgHeight < initialAvgHeight);
    // ...and water spreads out across the floor in x (mirrors the 2D dam
    // break test). It's seeded already spanning most of domainDepth in z,
    // so z-spread isn't expected to grow the same way — but it must not
    // collapse to a sliver either (particles should still be distributed
    // across roughly their initial z extent).
    REQUIRE(finalSpreadX > initialSpreadX);
    REQUIRE(finalSpreadZ > initialSpreadZ * 0.5f);

    // No explosion: velocities stay within a sane bound (water settling
    // under gravity in a small box shouldn't reach tens of m/s).
    for (std::size_t i = 0; i < n; ++i) {
        float vx = component(vel, i, 0);
        float vy = component(vel, i, 1);
        float vz = component(vel, i, 2);
        float speed = std::sqrt(vx * vx + vy * vy + vz * vz);
        REQUIRE(speed < 20.0f);
    }
}

TEST_CASE("3D dam break settles into a calm resting state and stays calm over a long run",
          "[solver3d][dambreak][regression]") {
    // 3D sibling of the 2D SPH "random launch" regression test
    // (tests/core/test_solver.cpp) — baked in from day one per the project's
    // explicit lesson-learned, not added after rediscovering the bug. Uses
    // the frontend's actual substep size (1/120s) and runs long enough
    // (25s) to pass well beyond the initial collapse transient and reach +
    // stay in the resting-bed regime where the 2D bug manifested.
    SphSimulation3D sim;
    Scenario3D scenario = fluids::makeDamBreak3DScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float dt = 1.0f / 120.0f;
    const int windowSteps = 120; // one simulated second per window
    const int windowCount = 25;
    const int totalSteps = windowSteps * windowCount;

    std::vector<float> windowMaxRms;
    float runningWindowMax = 0.0f;

    for (int step = 0; step < totalSteps; ++step) {
        sim.step(dt);

        const float* pos = sim.getPositionsPtr();
        const float* vel = sim.getVelocitiesPtr();
        REQUIRE(allFinite(pos, n * 3));
        REQUIRE(allFinite(vel, n * 3));

        runningWindowMax = std::max(runningWindowMax, rmsSpeed(vel, n));

        if ((step + 1) % windowSteps == 0) {
            windowMaxRms.push_back(runningWindowMax);
            runningWindowMax = 0.0f;
        }
    }

    REQUIRE(windowMaxRms.size() == static_cast<std::size_t>(windowCount));

    // Back half of the run should stay calm in EVERY window, not just on
    // average (see the 2D test's comment for why "on average" isn't enough).
    const std::size_t backHalfStart = windowMaxRms.size() / 2;
    for (std::size_t i = backHalfStart; i < windowMaxRms.size(); ++i) {
        INFO("window " << i << " (t=[" << i << "," << i + 1 << "]s) peak RMS speed = " << windowMaxRms[i]);
        REQUIRE(windowMaxRms[i] < 2.0f);
    }

    // Trend check: should not be getting MORE energetic over time.
    float earlySettledPeak = *std::max_element(windowMaxRms.begin() + 2, windowMaxRms.begin() + 4);
    float backHalfPeak = *std::max_element(windowMaxRms.begin() + static_cast<long>(backHalfStart), windowMaxRms.end());
    REQUIRE(backHalfPeak <= earlySettledPeak * 1.5f);
}
