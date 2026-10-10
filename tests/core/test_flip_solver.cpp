#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "fluids/FlipSimulation.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/flip/FlipParams.h"
#include "fluids/solvers/flip/FlipSolver.h"

using fluids::FlipSimulation;
using fluids::Scenario;
using fluids::flip::CellType;
using fluids::flip::FlipParams;
using fluids::flip::FlipSolver;

namespace {

bool allFinite(const float* buf, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(buf[i])) return false;
    }
    return true;
}

float averageHeight(const float* positions, std::size_t particleCount) {
    if (particleCount == 0) return 0.0f;
    double sum = 0.0;
    for (std::size_t i = 0; i < particleCount; ++i) {
        sum += positions[i * 2 + 1]; // y component
    }
    return static_cast<float>(sum / static_cast<double>(particleCount));
}

// A scenario that densely fills most of a small square domain (leaving a
// margin so most seeded particles land a few cells away from the Solid
// boundary ring), for grid-transfer/projection tests that want the field to
// be well-defined deep in the interior.
Scenario makeDenseFillScenario(float vx, float vy) {
    Scenario s;
    s.domainWidth = 1.0f;
    s.domainHeight = 1.0f;
    s.spacing = 0.04f;
    s.regionX = 0.1f;
    s.regionY = 0.1f;
    s.regionWidth = 0.8f;
    s.regionHeight = 0.8f;
    s.velocityX = vx;
    s.velocityY = vy;
    s.hasRegion2 = false;
    return s;
}

} // namespace

TEST_CASE("FlipSimulation default construction seeds particles", "[flip][smoke]") {
    FlipSimulation sim;
    REQUIRE(sim.getParticleCount() > 0);
}

TEST_CASE("P2G then G2P with flipRatio=0 (pure PIC) roughly reproduces a uniform velocity field",
          "[flip][transfer]") {
    FlipSolver solver;
    FlipParams params;
    params.gravity = 0.0f; // isolate the transfer itself from gravity's effect
    params.flipRatio = 0.0f; // pure PIC: particle velocity <- interpolated grid velocity
    params.cellSize = 0.05f;

    const float vx = 1.3f;
    const float vy = -0.7f;
    Scenario scenario = makeDenseFillScenario(vx, vy);
    solver.reset(scenario, params);
    REQUIRE(solver.particleCount() > 0);

    solver.step(params, 1.0f / 240.0f);

    // A uniform velocity field is already divergence-free in the interior,
    // so pressure projection should leave it essentially unchanged there;
    // only particles near the Solid boundary ring (where the wall-normal
    // component is legitimately zeroed) should deviate. Check particles
    // sitting at least 2 cells away from every wall.
    const float h = params.cellSize;
    const float margin = 2.0f * h;
    std::size_t checked = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        const auto& p = solver.positions()[i];
        if (p.x < margin || p.x > scenario.domainWidth - margin || p.y < margin ||
            p.y > scenario.domainHeight - margin) {
            continue;
        }
        const auto& v = solver.velocities()[i];
        REQUIRE(std::abs(v.x - vx) < 0.05f);
        REQUIRE(std::abs(v.y - vy) < 0.05f);
        ++checked;
    }
    REQUIRE(checked > 0); // sanity: the interior margin filter actually kept some particles
}

TEST_CASE("Pressure projection reduces divergence to near zero in interior fluid cells", "[flip][projection]") {
    FlipSolver solver;
    FlipParams params;
    params.gravity = -9.81f;
    params.flipRatio = 0.95f;
    params.cellSize = 0.05f;

    Scenario scenario = makeDenseFillScenario(0.0f, 0.0f);
    solver.reset(scenario, params);
    REQUIRE(solver.particleCount() > 0);

    const float dt = 1.0f / 240.0f;
    for (int i = 0; i < 5; ++i) {
        solver.step(params, dt);
    }

    // solver.grid() reflects the state right after the last step()'s
    // pressure projection (P2G for the *next* step hasn't run yet), so its
    // divergence should be near zero in every fluid cell.
    const auto& grid = solver.grid();
    int fluidCellsChecked = 0;
    for (int j = 0; j < grid.ny; ++j) {
        for (int i = 0; i < grid.nx; ++i) {
            if (grid.cell(i, j) != CellType::Fluid) continue;
            const float divergence =
                (grid.U(i + 1, j) - grid.U(i, j) + grid.V(i, j + 1) - grid.V(i, j)) / grid.h;
            REQUIRE(std::abs(divergence) < 0.1f);
            ++fluidCellsChecked;
        }
    }
    REQUIRE(fluidCellsChecked > 0);
}

TEST_CASE("FLIP solver stability smoke test: no NaN/Inf, stays within domain bounds", "[flip][smoke]") {
    FlipSimulation sim;
    Scenario scenario = fluids::makeDamBreakScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float dt = 1.0f / 240.0f;
    const int steps = 240 * 3; // 3 seconds
    const float margin = sim.params.cellSize; // small slack for boundary overshoot within a substep

    for (int i = 0; i < steps; ++i) {
        sim.step(dt);

        const float* pos = sim.getPositionsPtr();
        const float* vel = sim.getVelocitiesPtr();
        REQUIRE(allFinite(pos, n * 2));
        REQUIRE(allFinite(vel, n * 2));

        for (std::size_t p = 0; p < n; ++p) {
            const float x = pos[p * 2 + 0];
            const float y = pos[p * 2 + 1];
            REQUIRE(x >= -margin);
            REQUIRE(x <= scenario.domainWidth + margin);
            REQUIRE(y >= -margin);
            REQUIRE(y <= scenario.domainHeight + margin);
        }
    }
}

TEST_CASE("Dam break scenario settles into a flatter, lower profile under FLIP", "[flip][dambreak]") {
    FlipSimulation sim;
    Scenario scenario = fluids::makeDamBreakScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float initialAvgHeight = averageHeight(sim.getPositionsPtr(), n);
    const float initialMaxX = [&] {
        const float* pos = sim.getPositionsPtr();
        float maxX = 0.0f;
        for (std::size_t i = 0; i < n; ++i) maxX = std::max(maxX, pos[i * 2 + 0]);
        return maxX;
    }();
    const float initialHeightSpread = [&] {
        const float* pos = sim.getPositionsPtr();
        float minY = scenario.domainHeight, maxY = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            minY = std::min(minY, pos[i * 2 + 1]);
            maxY = std::max(maxY, pos[i * 2 + 1]);
        }
        return maxY - minY;
    }();

    const float dt = 1.0f / 240.0f;
    const int steps = 240 * 3; // 3 seconds of simulated time
    for (int i = 0; i < steps; ++i) {
        sim.step(dt);
    }

    const float* pos = sim.getPositionsPtr();
    const float* vel = sim.getVelocitiesPtr();
    REQUIRE(allFinite(pos, n * 2));
    REQUIRE(allFinite(vel, n * 2));

    const float finalAvgHeight = averageHeight(pos, n);
    float finalMaxX = 0.0f;
    float finalMinY = scenario.domainHeight;
    float finalMaxY = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        finalMaxX = std::max(finalMaxX, pos[i * 2 + 0]);
        finalMinY = std::min(finalMinY, pos[i * 2 + 1]);
        finalMaxY = std::max(finalMaxY, pos[i * 2 + 1]);
    }
    const float finalHeightSpread = finalMaxY - finalMinY;

    // The column collapses: average particle height drops...
    REQUIRE(finalAvgHeight < initialAvgHeight);
    // ...water spreads out across the floor (max x increases well past the
    // initial column width)...
    REQUIRE(finalMaxX > initialMaxX);
    // ...and the profile is visibly flatter (vertical extent shrinks) than
    // the initial tall column.
    REQUIRE(finalHeightSpread < initialHeightSpread);

    // No explosion: velocities stay within a sane bound.
    for (std::size_t i = 0; i < n; ++i) {
        const float speed = std::sqrt(vel[i * 2 + 0] * vel[i * 2 + 0] + vel[i * 2 + 1] * vel[i * 2 + 1]);
        REQUIRE(speed < 20.0f);
    }
}
