#include <algorithm>
#include <cmath>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>

#include "fluids/MpmSmokeSimulation.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeParams.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeSolver.h"

using fluids::MpmSmokeSimulation;
using fluids::SmokeScenario;
using fluids::mpm_smoke::MpmSmokeParams;
using fluids::mpm_smoke::MpmSmokeSolver;

namespace {

bool allFinite(const float* buf, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(buf[i])) return false;
    }
    return true;
}

} // namespace

TEST_CASE("MpmSmokeSolver reset allocates the fixed-capacity buffers at expected size", "[mpm_smoke][construction]") {
    MpmSmokeSolver solver;
    MpmSmokeParams params;
    params.maxParticles = 500;

    SmokeScenario scenario = fluids::makePlumeScenario();
    solver.reset(scenario, params);

    REQUIRE(solver.particleCount() == 0);
    REQUIRE(solver.maxParticleCapacity() == 500);
    REQUIRE(solver.positions().size() == 500);
    REQUIRE(solver.velocities().size() == 500);
    REQUIRE(solver.density().size() == 500);
    REQUIRE(solver.temperature().size() == 500);
}

TEST_CASE("MpmSmokeSimulation default construction seeds a live (initially empty-particle) sim",
          "[mpm_smoke][construction]") {
    MpmSmokeSimulation sim;
    // No particles exist the instant after reset() (emission happens in
    // step(), not reset()) but the buffers must already be valid/non-null.
    REQUIRE(sim.getParticleCount() == 0);
    REQUIRE(sim.getPositionsPtr() != nullptr);
    REQUIRE(sim.getVelocitiesPtr() != nullptr);
    REQUIRE(sim.getDensitiesPtr() != nullptr);
    REQUIRE(sim.getTemperaturesPtr() != nullptr);
}

TEST_CASE("Emission increases active particle count over the first several steps", "[mpm_smoke][emission]") {
    MpmSmokeSimulation sim;

    const float dt = 1.0f / 60.0f;
    std::size_t previous = sim.getParticleCount();
    REQUIRE(previous == 0);

    bool sawIncrease = false;
    for (int i = 0; i < 60; ++i) { // 1 simulated second
        sim.step(dt);
        const std::size_t current = sim.getParticleCount();
        REQUIRE(current >= previous); // never decreases this early (nothing has faded below threshold yet)
        if (current > previous) sawIncrease = true;
        previous = current;
    }
    REQUIRE(sawIncrease);
    REQUIRE(sim.getParticleCount() > 0);
}

TEST_CASE("Emission respects maxParticles as a hard cap", "[mpm_smoke][emission]") {
    MpmSmokeSimulation sim;
    MpmSmokeParams p = sim.params;
    p.maxParticles = 50;
    p.densityDissipation = 0.0f; // nothing fades away, isolating the emission-vs-cap behavior
    sim.params = p;
    sim.reset(fluids::makePlumeScenario());

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) { // 10 simulated seconds — plenty of time to fill 50 slots
        sim.step(dt);
        REQUIRE(sim.getParticleCount() <= 50);
    }
    // With emission active, dissipation off, and no culling possible, the
    // buffer should actually reach its cap rather than stall out early.
    REQUIRE(sim.getParticleCount() == 50);
}

TEST_CASE("Culling removes particles whose density has decayed below threshold", "[mpm_smoke][culling]") {
    // Synthetic pre-seeded batch (via the test-only debugSpawnParticle
    // helper), no emission at all (zero-area source), isolating culling
    // behavior from emission timing — per the task's suggested alternative
    // to waiting on natural emission+dissipation.
    MpmSmokeSolver solver;
    MpmSmokeParams params;
    params.densityDissipation = 5.0f; // fast decay so the threshold is crossed within a couple of seconds
    params.temperatureDissipation = 0.0f;
    params.buoyancy = 0.0f;
    params.maxParticles = 200;

    SmokeScenario scenario = fluids::makePlumeScenario();
    scenario.sourceWidth = 0.0f; // disable emission entirely for this test
    scenario.sourceHeight = 0.0f;
    solver.reset(scenario, params);

    const int seeded = 100;
    for (int i = 0; i < seeded; ++i) {
        REQUIRE(solver.debugSpawnParticle(fluids::Vec2(0.5f, 1.0f), fluids::Vec2(0.0f, 0.0f), 0.5f, 0.0f));
    }
    REQUIRE(solver.particleCount() == static_cast<std::size_t>(seeded));

    const float dt = 1.0f / 60.0f;
    // ln(0.5/0.01)/5 ~= 0.78s to cross the 0.01 threshold at this decay
    // rate; run well past that.
    for (int i = 0; i < 180; ++i) { // 3 simulated seconds
        solver.step(params, dt);
    }

    REQUIRE(solver.particleCount() == 0);
}

TEST_CASE("Culling keeps the active population bounded rather than growing forever under sustained emission",
          "[mpm_smoke][culling]") {
    MpmSmokeSimulation sim; // default params: emission on, dissipation on, maxParticles=4000

    const float dt = 1.0f / 60.0f;
    std::size_t maxSeen = 0;
    for (int i = 0; i < 60 * 30; ++i) { // 30 simulated seconds
        sim.step(dt);
        maxSeen = std::max(maxSeen, sim.getParticleCount());
    }
    // Should have reached a bounded population comfortably under the
    // maxParticles safety cap (4000 by default) — culling is doing its job,
    // not just relying on the hard cap to stop growth. See
    // docs/mpm-smoke-sim-api.md for the measured steady-state count.
    REQUIRE(maxSeen > 0);
    REQUIRE(maxSeen < 4000);
}

TEST_CASE("Buoyancy drives net upward velocity for newly-emitted hot particles, more than ambient-temperature ones",
          "[mpm_smoke][buoyancy]") {
    auto averageVy = [](MpmSmokeSimulation& sim, int steps, float dt) {
        for (int i = 0; i < steps; ++i) sim.step(dt);
        const std::size_t n = sim.getParticleCount();
        REQUIRE(n > 0);
        const float* vel = sim.getVelocitiesPtr();
        double sum = 0.0;
        for (std::size_t i = 0; i < n; ++i) sum += vel[i * 2 + 1];
        return static_cast<float>(sum / static_cast<double>(n));
    };

    const float dt = 1.0f / 60.0f;
    const int steps = 30; // 0.5 simulated seconds

    MpmSmokeSimulation hot;
    MpmSmokeParams hotParams = hot.params;
    hotParams.ambientTemperature = 0.0f;
    hotParams.sourceTemperature = 1.0f;
    hotParams.buoyancy = 6.0f;
    hot.params = hotParams;
    hot.reset(fluids::makePlumeScenario());
    const float hotAvgVy = averageVy(hot, steps, dt);

    MpmSmokeSimulation ambient;
    MpmSmokeParams ambientParams = ambient.params;
    ambientParams.ambientTemperature = 0.0f;
    ambientParams.sourceTemperature = 0.0f; // no temperature difference at all -> no buoyancy force
    ambientParams.buoyancy = 6.0f;
    ambient.params = ambientParams;
    ambient.reset(fluids::makePlumeScenario());
    const float ambientAvgVy = averageVy(ambient, steps, dt);

    REQUIRE(hotAvgVy > 0.0f);
    REQUIRE(hotAvgVy > ambientAvgVy);
}

TEST_CASE("Plume preset stability: no NaN/Inf, particles stay near domain bounds, active count stays in range",
          "[mpm_smoke][stability]") {
    MpmSmokeSimulation sim;
    SmokeScenario scenario = fluids::makePlumeScenario();
    sim.reset(scenario);

    const float dt = 1.0f / 60.0f;
    const int steps = 60 * 5; // 5 simulated seconds
    const float margin = sim.params.cellSize * 4.0f;
    const std::size_t capacity = static_cast<std::size_t>(sim.params.maxParticles);

    for (int i = 0; i < steps; ++i) {
        sim.step(dt);

        const std::size_t n = sim.getParticleCount();
        REQUIRE(n <= capacity);

        const float* pos = sim.getPositionsPtr();
        const float* vel = sim.getVelocitiesPtr();
        const float* density = sim.getDensitiesPtr();
        const float* temperature = sim.getTemperaturesPtr();
        REQUIRE(allFinite(pos, n * 2));
        REQUIRE(allFinite(vel, n * 2));
        REQUIRE(allFinite(density, n));
        REQUIRE(allFinite(temperature, n));

        for (std::size_t p = 0; p < n; ++p) {
            const float x = pos[p * 2 + 0];
            const float y = pos[p * 2 + 1];
            REQUIRE(x >= -margin);
            REQUIRE(x <= scenario.domainWidth + margin);
            REQUIRE(y >= -margin);
            REQUIRE(y <= scenario.domainHeight + margin);
        }
    }

    // Manual-trace-style sanity check (mirrors FlipSmokeSolver's stability
    // test): the density-weighted centroid height of the active particles
    // should have risen above the source region by now.
    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);
    const float* pos = sim.getPositionsPtr();
    const float* density = sim.getDensitiesPtr();
    double weightedY = 0.0;
    double totalD = 0.0;
    for (std::size_t p = 0; p < n; ++p) {
        weightedY += static_cast<double>(density[p]) * static_cast<double>(pos[p * 2 + 1]);
        totalD += density[p];
    }
    REQUIRE(totalD > 0.0);
    const double centroidY = weightedY / totalD;
    REQUIRE(centroidY > scenario.sourceY + scenario.sourceHeight);
}

TEST_CASE("Buffer pointers are stable across step() calls despite particle count changing",
          "[mpm_smoke][pointer_stability]") {
    MpmSmokeSimulation sim;
    sim.reset(fluids::makePlumeScenario());

    const float* posPtr = sim.getPositionsPtr();
    const float* velPtr = sim.getVelocitiesPtr();
    const float* densPtr = sim.getDensitiesPtr();
    const float* tempPtr = sim.getTemperaturesPtr();

    const float dt = 1.0f / 60.0f;
    std::size_t firstCount = sim.getParticleCount();
    bool sawCountChange = false;

    for (int i = 0; i < 600; ++i) { // 10 simulated seconds
        sim.step(dt);
        if (sim.getParticleCount() != firstCount) sawCountChange = true;

        REQUIRE(sim.getPositionsPtr() == posPtr);
        REQUIRE(sim.getVelocitiesPtr() == velPtr);
        REQUIRE(sim.getDensitiesPtr() == densPtr);
        REQUIRE(sim.getTemperaturesPtr() == tempPtr);
    }

    // Sanity: the count actually did change over this run (otherwise the
    // pointer-stability check above would be vacuous — of course an unused
    // pointer stays the same).
    REQUIRE(sawCountChange);

    // reset() is allowed to (and does) reallocate — confirm the contract's
    // other half: after reset(), a NEW fetch is required, and pointers may
    // differ from before (not asserting they must differ, just that the
    // API doesn't silently invalidate the old ones in a way tests would
    // catch as UB — re-fetching and using immediately after reset is the
    // documented contract).
    sim.reset(fluids::makePlumeScenario());
    const float* posPtrAfterReset = sim.getPositionsPtr();
    REQUIRE(posPtrAfterReset != nullptr);
}

TEST_CASE("Fixed-substep stepping is stable across a range of caller dt values (no adaptive CFL subdivision needed)",
          "[mpm_smoke][stability][substep]") {
    // Unlike mpm::MpmSolver (water), this solver has no stress/pressure
    // feedback loop at all (see MpmSmokeSolver's class comment), so the
    // specific runaway mode that forced water's adaptive CFL substepping
    // (a stiff EOS amplifying a large J swing at a sudden impact) cannot
    // occur here. This test checks directly whether a fast emitted-particle
    // stream could still, in principle, destabilize a plain fixed-substep
    // scheme at a larger caller dt — rather than assuming either prior
    // finding (MPM water's 1/480 requirement, or FLIP smoke's "any dt up to
    // 1/30 is fine") transfers to this genuinely different solver.
    const float dts[] = {1.0f / 30.0f, 1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 240.0f};
    const float maxSaneSpeed = 50.0f; // generous — real speeds here should be a small fraction of this

    for (float dt : dts) {
        MpmSmokeSimulation sim;
        SmokeScenario scenario = fluids::makePlumeScenario();
        sim.reset(scenario);

        const int steps = static_cast<int>(3.0f / dt); // 3 simulated seconds at this dt
        for (int i = 0; i < steps; ++i) {
            sim.step(dt);

            const std::size_t n = sim.getParticleCount();
            const float* pos = sim.getPositionsPtr();
            const float* vel = sim.getVelocitiesPtr();
            REQUIRE(allFinite(pos, n * 2));
            REQUIRE(allFinite(vel, n * 2));

            for (std::size_t p = 0; p < n; ++p) {
                const float vx = vel[p * 2 + 0];
                const float vy = vel[p * 2 + 1];
                const float speed = std::sqrt(vx * vx + vy * vy);
                REQUIRE(speed < maxSaneSpeed);
            }
        }
    }
}
