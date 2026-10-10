#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "fluids/MpmSimulation.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/mpm/MpmParams.h"
#include "fluids/solvers/mpm/MpmSolver.h"

using fluids::MpmSimulation;
using fluids::Scenario;
using fluids::mpm::MpmParams;
using fluids::mpm::MpmSolver;

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
// margin so most seeded particles land a few cells away from the boundary
// velocity-clamp band), for transfer tests that want the field to be
// well-defined deep in the interior. Mirrors FLIP's
// makeDenseFillScenario helper (tests/core/test_flip_solver.cpp).
Scenario makeDenseFillScenario(float vx, float vy) {
    Scenario s;
    s.domainWidth = 1.0f;
    s.domainHeight = 1.0f;
    s.spacing = 0.025f;
    s.regionX = 0.15f;
    s.regionY = 0.15f;
    s.regionWidth = 0.7f;
    s.regionHeight = 0.7f;
    s.velocityX = vx;
    s.velocityY = vy;
    s.hasRegion2 = false;
    return s;
}

} // namespace

TEST_CASE("MpmSimulation default construction seeds particles", "[mpm][smoke]") {
    MpmSimulation sim;
    REQUIRE(sim.getParticleCount() > 0);
}

TEST_CASE("P2G then G2P with a uniform velocity field (no gravity/stress) roughly reproduces it",
          "[mpm][transfer]") {
    MpmSolver solver;
    MpmParams params;
    params.gravity = 0.0f; // isolate the transfer itself from gravity
    params.stiffness = 0.0f; // and from any EOS pressure force (flat/uniform state, nothing should move)
    params.viscosity = 0.0f;
    params.cellSize = 0.05f;

    const float vx = 0.8f;
    const float vy = -0.4f;
    Scenario scenario = makeDenseFillScenario(vx, vy);
    solver.reset(scenario, params);
    REQUIRE(solver.particleCount() > 0);

    solver.step(params, 1.0f / 240.0f);

    // With stiffness=0 (no pressure force) and gravity=0, a uniform velocity
    // field should transfer to the grid and back with only the boundary
    // velocity clamp perturbing particles very close to a wall. Check
    // particles sitting well inside the domain (away from the boundary
    // clamp band).
    const float h = params.cellSize;
    const float margin = 4.0f * h;
    std::size_t checked = 0;
    for (std::size_t i = 0; i < solver.particleCount(); ++i) {
        const auto& p = solver.positions()[i];
        if (p.x < margin || p.x > scenario.domainWidth - margin || p.y < margin ||
            p.y > scenario.domainHeight - margin) {
            continue;
        }
        const auto& v = solver.velocities()[i];
        REQUIRE(std::abs(v.x - vx) < 0.02f);
        REQUIRE(std::abs(v.y - vy) < 0.02f);
        ++checked;
    }
    REQUIRE(checked > 0); // sanity: the interior margin filter actually kept some particles
}

TEST_CASE("J stays within documented bounds and doesn't drift to a degenerate value for a settled configuration",
          "[mpm][stability]") {
    // A shallow, wide layer already resting near the floor (not a
    // freshly-released tall column) — chosen deliberately over
    // makeDamBreakScenario() for this test because a dam break is still
    // actively splashing/spreading at the 3-second mark (see the dedicated
    // dambreak settling test below), which is a much noisier regime for a
    // per-particle J check than an already-at-rest layer. This scenario
    // reaches quiescence quickly under gravity and stays there.
    Scenario scenario;
    scenario.domainWidth = 2.0f;
    scenario.domainHeight = 1.5f;
    scenario.spacing = 0.03f;
    scenario.regionX = 0.1f;
    scenario.regionY = 0.05f;
    scenario.regionWidth = 1.8f;
    scenario.regionHeight = 0.15f;
    scenario.velocityX = 0.0f;
    scenario.velocityY = 0.0f;
    scenario.hasRegion2 = false;

    MpmSolver solver;
    MpmParams params; // defaults: stiffness=20, restDensity=1000, cellSize=0.02
    solver.reset(scenario, params);

    const float dt = 1.0f / 480.0f; // see docs/mpm-sim-api.md for how this substep was determined
    const int steps = static_cast<int>(1.5f / dt);
    for (int i = 0; i < steps; ++i) {
        solver.step(params, dt);
    }

    const auto& J = solver.J();
    REQUIRE(J.size() > 0);
    for (float j : J) {
        REQUIRE(std::isfinite(j));
        // Documented clamp range (see MpmSolver.cpp's kJMin/kJMax) — must
        // never be exceeded.
        REQUIRE(j >= 0.1f);
        REQUIRE(j <= 10.0f);
        // Empirically (see this test's development), a settled shallow
        // layer under the default stiffness compresses to roughly
        // J ~= 0.55-0.95 (the weakly-compressible EOS lets the resting
        // hydrostatic load compress the material somewhat before pressure
        // balances gravity — the same qualitative behavior WCSPH shows at
        // moderate stiffness) and does not drift further once settled.
        // These bounds are well inside the [0.1, 10] clamp and catch any
        // future regression that lets J run away toward either extreme.
        REQUIRE(j > 0.35f);
        REQUIRE(j < 1.3f);
    }
}

TEST_CASE("MPM solver stability smoke test: no NaN/Inf, stays within domain bounds", "[mpm][smoke]") {
    MpmSimulation sim;
    Scenario scenario = fluids::makeDamBreakScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    // See docs/mpm-sim-api.md for how this substep was determined
    // empirically (explicit MPM with this default stiffness needs a
    // smaller substep than SPH/FLIP's 1/240).
    const float dt = 1.0f / 480.0f;
    const int steps = 480 * 3; // 3 seconds
    const float margin = sim.params.cellSize * 4.0f; // slack for boundary overshoot within a substep

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

TEST_CASE("Block Drop scenario is stable through impact and settles without exploding",
          "[mpm][blockdrop][stability]") {
    // Regression test for the Block-Drop-specific explosion this solver used
    // to produce: makeBlockDropScenario() seeds a square block well above
    // the floor (regionY=1.2 in a 2.0-tall domain), so it free-falls as a
    // single coherent mass and hits the floor at once at roughly
    // sqrt(2*9.81*1.2) ~= 4.9 m/s, unlike Dam Break/Two-Column Collision
    // where material starts at rest or collides more gradually. Before the
    // internal adaptive-substep fix (see MpmSolver::step), this impact
    // produced a velocity gradient across the impact interface large enough
    // to swing a particle's J to the kJMin=0.1 clamp within a single
    // dt=1/480 step, which the stiff EOS then amplified into a
    // hundreds-of-m/s runaway within a handful of further steps (confirmed
    // via a native trace of max particle speed / J bounds during
    // development of this fix — see docs/mpm-sim-api.md).
    MpmSimulation sim;
    Scenario scenario = fluids::makeBlockDropScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    // Same substep the frontend actually drives the solver with (see
    // docs/mpm-sim-api.md) — the fix must keep this scenario stable at the
    // documented dt, not require a different one.
    const float dt = 1.0f / 480.0f;
    const int steps = static_cast<int>(4.0f / dt); // well past impact (~0.5s) and settling
    const float margin = sim.params.cellSize * 4.0f; // slack for boundary overshoot within a substep

    // A physically plausible upper bound on speed anywhere in this
    // simulation: free-fall from the block's starting height plus generous
    // headroom for splash acceleration from the pressure force at impact.
    // The pre-fix bug blew this bound away within a couple of steps
    // (velocities in the hundreds to low thousands of m/s); a stable solver
    // should never approach it.
    const float maxSaneSpeed = 20.0f;

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

            const float vx = vel[p * 2 + 0];
            const float vy = vel[p * 2 + 1];
            const float speed = std::sqrt(vx * vx + vy * vy);
            REQUIRE(speed < maxSaneSpeed);
        }
    }

    // After 4 seconds the block should have landed and be settling, not
    // still scattered at high speed: average height should have dropped
    // well below the block's initial seed height (regionY=1.2 + half the
    // block's own height).
    const float finalAvgHeight = averageHeight(sim.getPositionsPtr(), n);
    REQUIRE(finalAvgHeight < scenario.regionY);
}

TEST_CASE("Dam break scenario settles into a flatter, lower profile under MPM", "[mpm][dambreak]") {
    MpmSimulation sim;
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

    const float dt = 1.0f / 480.0f;
    const int steps = 480 * 3; // 3 seconds of simulated time
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
