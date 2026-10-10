#include <algorithm>
#include <cmath>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "fluids/Simulation.h"
#include "fluids/scenario/Scenario.h"

using fluids::Scenario;
using fluids::Simulation;

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

} // namespace

TEST_CASE("Simulation default construction seeds particles", "[solver][smoke]") {
    Simulation sim;
    REQUIRE(sim.getParticleCount() > 0);
}

TEST_CASE("Solver stability smoke test: no NaN/Inf, stays within domain bounds", "[solver][smoke]") {
    Simulation sim;
    Scenario scenario = fluids::makeDamBreakScenario();
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
        REQUIRE(allFinite(pos, n * 2));
        REQUIRE(allFinite(vel, n * 2));

        for (std::size_t p = 0; p < n; ++p) {
            float x = pos[p * 2 + 0];
            float y = pos[p * 2 + 1];
            REQUIRE(x >= -margin);
            REQUIRE(x <= scenario.domainWidth + margin);
            REQUIRE(y >= -margin);
            REQUIRE(y <= scenario.domainHeight + margin);
        }
    }
}

TEST_CASE("Dam break scenario collapses and settles under gravity", "[solver][dambreak]") {
    Simulation sim;
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
    for (std::size_t i = 0; i < n; ++i) finalMaxX = std::max(finalMaxX, pos[i * 2 + 0]);

    // The column collapses: average particle height drops...
    REQUIRE(finalAvgHeight < initialAvgHeight);
    // ...and water spreads out across the floor (max x increases well past
    // the initial column width).
    REQUIRE(finalMaxX > initialMaxX);

    // No explosion: velocities stay within a sane bound (water settling
    // under gravity in a small box shouldn't reach tens of m/s).
    for (std::size_t i = 0; i < n; ++i) {
        float speed = std::sqrt(vel[i * 2 + 0] * vel[i * 2 + 0] + vel[i * 2 + 1] * vel[i * 2 + 1]);
        REQUIRE(speed < 20.0f);
    }
}

namespace {

// Root-mean-square speed across all particles — a robust "how much is
// everything jittering" signal (unlike max speed, one persistently-launched
// particle can't dominate it, and unlike average velocity it doesn't cancel
// out particles moving in opposite directions).
float rmsSpeed(const float* vel, std::size_t n) {
    if (n == 0) return 0.0f;
    double sumSq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        float vx = vel[i * 2 + 0];
        float vy = vel[i * 2 + 1];
        sumSq += static_cast<double>(vx) * vx + static_cast<double>(vy) * vy;
    }
    return static_cast<float>(std::sqrt(sumSq / static_cast<double>(n)));
}

} // namespace

TEST_CASE("Dam break settles into a calm resting state and stays calm over a long run",
          "[solver][dambreak][regression]") {
    // Regression test for a bug where a resting bed of particles never
    // actually went quiet: individual particles would be launched to
    // double-digit m/s indefinitely (a persistent, non-decaying "rain"),
    // driven by a handful of near-coincident neighbor pairs producing
    // wildly disproportionate density -> pressure (Tait EOS, exponent 7)
    // -> force spikes each time one of them nudged apart slightly. Fixed by
    // flooring the effective interaction distance used inside the poly6 and
    // spiky kernels (see kMinEffectiveRFraction in Kernels.h) — this test
    // would have failed against the pre-fix solver (RMS speed kept spiking
    // back up throughout the run instead of settling).
    //
    // Uses the frontend's actual substep size (1/120s, per docs/sim-api.md)
    // and runs long enough (25s) to pass well beyond the initial collapse
    // transient (~1.5s, per docs/sim-api.md) and reach + stay in the
    // resting-bed regime where the bug manifested.
    Simulation sim;
    Scenario scenario = fluids::makeDamBreakScenario();
    sim.reset(scenario);

    const std::size_t n = sim.getParticleCount();
    REQUIRE(n > 0);

    const float dt = 1.0f / 120.0f;
    const int windowSteps = 120; // one simulated second per window
    const int windowCount = 25;
    // Computed from windowSteps/windowCount (not float division of a target
    // duration by dt) so it's exactly windowCount * windowSteps regardless
    // of dt's float rounding — otherwise 25.0f/dt can truncate to one step
    // short of a full last window.
    const int totalSteps = windowSteps * windowCount;

    std::vector<float> windowMaxRms;
    float runningWindowMax = 0.0f;

    for (int step = 0; step < totalSteps; ++step) {
        sim.step(dt);

        const float* pos = sim.getPositionsPtr();
        const float* vel = sim.getVelocitiesPtr();
        REQUIRE(allFinite(pos, n * 2));
        REQUIRE(allFinite(vel, n * 2));

        runningWindowMax = std::max(runningWindowMax, rmsSpeed(vel, n));

        if ((step + 1) % windowSteps == 0) {
            windowMaxRms.push_back(runningWindowMax);
            runningWindowMax = 0.0f;
        }
    }

    REQUIRE(windowMaxRms.size() == static_cast<std::size_t>(windowCount));

    // The back half of the run (seconds 13-25, well past the ~1.5s initial
    // collapse) is the "should be resting" regime the bug affected: the old
    // solver kept producing recurring high-speed launches here forever,
    // while FLIP/MPM settle down and stay down. Assert every one-second
    // window's peak RMS speed back there stays low and bounded — not just
    // low on average (which a single persistent spike-then-recover pattern
    // could still satisfy), but low in EVERY window, which is exactly what
    // "never actually stops" fails.
    const std::size_t backHalfStart = windowMaxRms.size() / 2;
    for (std::size_t i = backHalfStart; i < windowMaxRms.size(); ++i) {
        INFO("window " << i << " (t=[" << i << "," << i + 1 << "]s) peak RMS speed = " << windowMaxRms[i]);
        // A calm resting bed under default params jitters at a fraction of
        // a m/s (RMS, not max); the pre-fix bug repeatedly drove this well
        // into the multi-m/s range from a single launched particle. 2.0
        // m/s gives generous headroom above normal jitter while still
        // being far below what even one bug-triggered launch produces.
        REQUIRE(windowMaxRms[i] < 2.0f);
    }

    // Trend check: the run should not be getting MORE energetic over time
    // (the bug's signature was a non-decaying, if anything self-reinforcing,
    // pattern as more near-coincident pairs accumulated). Compare the peak
    // of the back half against the peak of the window right after the
    // initial collapse settles (seconds 2-4) — it should not exceed it.
    float earlySettledPeak = *std::max_element(windowMaxRms.begin() + 2, windowMaxRms.begin() + 4);
    float backHalfPeak = *std::max_element(windowMaxRms.begin() + static_cast<long>(backHalfStart), windowMaxRms.end());
    REQUIRE(backHalfPeak <= earlySettledPeak * 1.5f);
}
