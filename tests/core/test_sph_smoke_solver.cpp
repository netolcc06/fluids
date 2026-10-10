#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "fluids/SphSmokeSimulation.h"
#include "fluids/common/UniformGrid.h"
#include "fluids/common/Vec2.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/sph_smoke/SphSmokeParams.h"
#include "fluids/solvers/sph_smoke/SphSmokeSolver.h"

using fluids::SphSmokeSimulation;
using fluids::SmokeScenario;
using fluids::UniformGrid;
using fluids::Vec2;
using fluids::sph_smoke::SphSmokeParams;
using fluids::sph_smoke::SphSmokeSolver;

namespace {

bool allFinite(const float* buf, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(buf[i])) return false;
    }
    return true;
}

// Root-mean-square speed across all active particles — same robust
// "how much is everything jittering" signal used by test_solver.cpp's water
// settling regression test. Well-defined even though the active population
// changes every step here (unlike water's fixed population) since it's
// already normalized by n.
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

// Average magnitude of each active particle's velocity minus the mean
// velocity of its own kernel-radius neighbors — a direct measurement of
// "how locally coherent is the motion" (low = neighbors move alike, high =
// each particle moves independently of its neighbors). Used to verify the
// viscosity term is doing genuine, measurable kernel-based coherence work
// (the "genuine SPH character" this solver is meant to have via
// viscosityLaplacian, in the absence of any pressure force) rather than
// being visually inert at the chosen default value.
double averageNeighborVelocityResidual(SphSmokeSimulation& sim) {
    const std::size_t n = sim.getParticleCount();
    if (n == 0) return 0.0;

    const float* posBuf = sim.getPositionsPtr();
    const float* velBuf = sim.getVelocitiesPtr();
    std::vector<Vec2> pos(n), vel(n);
    for (std::size_t i = 0; i < n; ++i) {
        pos[i] = Vec2(posBuf[i * 2 + 0], posBuf[i * 2 + 1]);
        vel[i] = Vec2(velBuf[i * 2 + 0], velBuf[i * 2 + 1]);
    }

    const float radius = sim.params.smoothingRadius;
    UniformGrid grid(radius);
    grid.build(pos);

    double totalResidual = 0.0;
    std::size_t counted = 0;
    std::vector<int> neighbors;
    for (std::size_t i = 0; i < n; ++i) {
        neighbors.clear();
        grid.queryNeighbors(pos[i], radius, neighbors);

        Vec2 meanNeighborVel(0.0f, 0.0f);
        int neighborCount = 0;
        for (int j : neighbors) {
            if (static_cast<std::size_t>(j) == i) continue;
            meanNeighborVel += vel[static_cast<std::size_t>(j)];
            ++neighborCount;
        }
        if (neighborCount == 0) continue;
        meanNeighborVel *= (1.0f / static_cast<float>(neighborCount));

        const Vec2 residual = vel[i] - meanNeighborVel;
        totalResidual += residual.length();
        ++counted;
    }
    return counted > 0 ? totalResidual / static_cast<double>(counted) : 0.0;
}

} // namespace

TEST_CASE("SphSmokeSolver reset allocates the fixed-capacity buffers at expected size", "[sph_smoke][construction]") {
    SphSmokeSolver solver;
    SphSmokeParams params;
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

TEST_CASE("SphSmokeSimulation default construction seeds a live (initially empty-particle) sim",
          "[sph_smoke][construction]") {
    SphSmokeSimulation sim;
    // No particles exist the instant after reset() (emission happens in
    // step(), not reset()) but the buffers must already be valid/non-null.
    REQUIRE(sim.getParticleCount() == 0);
    REQUIRE(sim.getPositionsPtr() != nullptr);
    REQUIRE(sim.getVelocitiesPtr() != nullptr);
    REQUIRE(sim.getDensitiesPtr() != nullptr);
    REQUIRE(sim.getTemperaturesPtr() != nullptr);
}

TEST_CASE("Emission increases active particle count over the first several steps", "[sph_smoke][emission]") {
    SphSmokeSimulation sim;

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

TEST_CASE("Emission respects maxParticles as a hard cap", "[sph_smoke][emission]") {
    SphSmokeSimulation sim;
    SphSmokeParams p = sim.params;
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

TEST_CASE("Culling removes particles whose density has decayed below threshold", "[sph_smoke][culling]") {
    // Synthetic pre-seeded batch (via the test-only debugSpawnParticle
    // helper), no emission at all (zero-area source), isolating culling
    // behavior from emission timing.
    SphSmokeSolver solver;
    SphSmokeParams params;
    params.densityDissipation = 5.0f; // fast decay so the threshold is crossed within a couple of seconds
    params.temperatureDissipation = 0.0f;
    params.buoyancy = 0.0f;
    params.viscosity = 0.0f;
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
          "[sph_smoke][culling]") {
    SphSmokeSimulation sim; // default params: emission on, dissipation on, maxParticles=4000

    const float dt = 1.0f / 60.0f;
    std::size_t maxSeen = 0;
    for (int i = 0; i < 60 * 30; ++i) { // 30 simulated seconds
        sim.step(dt);
        maxSeen = std::max(maxSeen, sim.getParticleCount());
    }
    // Should have reached a bounded population comfortably under the
    // maxParticles safety cap (4000 by default) — culling is doing its job,
    // not just relying on the hard cap to stop growth. See
    // docs/sph-smoke-sim-api.md for the measured steady-state count.
    REQUIRE(maxSeen > 0);
    REQUIRE(maxSeen < 4000);
}

TEST_CASE("Buoyancy drives net upward velocity for newly-emitted hot particles, more than ambient-temperature ones",
          "[sph_smoke][buoyancy]") {
    auto averageVy = [](SphSmokeSimulation& sim, int steps, float dt) {
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

    SphSmokeSimulation hot;
    SphSmokeParams hotParams = hot.params;
    hotParams.ambientTemperature = 0.0f;
    hotParams.sourceTemperature = 1.0f;
    hotParams.buoyancy = 6.0f;
    hot.params = hotParams;
    hot.reset(fluids::makePlumeScenario());
    const float hotAvgVy = averageVy(hot, steps, dt);

    SphSmokeSimulation ambient;
    SphSmokeParams ambientParams = ambient.params;
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
          "[sph_smoke][stability]") {
    SphSmokeSimulation sim;
    SmokeScenario scenario = fluids::makePlumeScenario();
    sim.reset(scenario);

    const float dt = 1.0f / 60.0f;
    const int steps = 60 * 5; // 5 simulated seconds
    const float margin = sim.params.smoothingRadius * 2.0f;
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

    // Manual-trace-style sanity check (mirrors the other two smoke
    // solvers' stability tests): the density-weighted centroid height of
    // the active particles should have risen above the source region by
    // now.
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
          "[sph_smoke][pointer_stability]") {
    SphSmokeSimulation sim;
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
    // pointer-stability check above would be vacuous).
    REQUIRE(sawCountChange);

    // reset() is allowed to (and does) reallocate — confirm the contract's
    // other half.
    sim.reset(fluids::makePlumeScenario());
    const float* posPtrAfterReset = sim.getPositionsPtr();
    REQUIRE(posPtrAfterReset != nullptr);
}

TEST_CASE("Fixed-substep stepping is stable across a range of caller dt values (no adaptive CFL subdivision needed)",
          "[sph_smoke][stability][substep]") {
    // This solver has no pressure/EOS feedback loop at all (see
    // SphSmokeSolver's class comment), so the specific runaway mode that
    // forced water SPH's near-coincidence kernel fix (Tait EOS exponent-7
    // amplification of an inflated poly6 density estimate) cannot occur —
    // neither poly6's density estimate nor spikyGradient's pressure force
    // is ever fed through an EOS here. This test checks directly whether a
    // fast emitted-particle stream could still destabilize a plain
    // fixed-substep viscosity-only scheme at a larger caller dt, rather
    // than assuming any prior solver's finding transfers.
    const float dts[] = {1.0f / 30.0f, 1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 240.0f};
    const float maxSaneSpeed = 50.0f; // generous — real speeds here should be a small fraction of this

    for (float dt : dts) {
        SphSmokeSimulation sim;
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

TEST_CASE("Long-run speed stays bounded with no persistent high-speed outliers "
          "(SPH near-singularity settling-bug regression, adapted to a continuously-changing population)",
          "[sph_smoke][stability][regression]") {
    // Regression check for the exact failure mode documented in
    // fluids::sph::Kernels.h / tests/core/test_solver.cpp's water settling
    // test: a handful of near-coincident neighbor pairs producing wildly
    // disproportionate kernel output, funneled through a stiff EOS, into a
    // force spike that launches individual particles to double-digit m/s
    // forever. This solver has no EOS/pressure force to amplify anything
    // that way (see class comment), so this is not expected to reproduce —
    // this test exists to actually verify that rather than assume it from
    // the architecture description alone.
    //
    // Unlike water's fixed population, this solver's active particle count
    // changes every step (continuous emission/culling), so per-particle
    // "is this ONE particle still calm" tracking isn't meaningful across
    // steps (particle indices are reused via swap-on-cull). Instead, this
    // uses the same windowed-peak-RMS-speed approach as the water
    // regression test: RMS speed is already population-size-normalized, so
    // it stays a well-defined, comparable signal even as n changes every
    // step. A persistent, non-decaying "rain" of launched particles (the
    // water bug's signature) would show up as elevated peak-RMS in every
    // window indefinitely; ordinary buoyant-plume dynamics should not.
    SphSmokeSimulation sim;
    sim.reset(fluids::makePlumeScenario());

    const float dt = 1.0f / 60.0f;
    const int windowSteps = 60; // one simulated second per window
    const int windowCount = 30; // 30 simulated seconds total
    const int totalSteps = windowSteps * windowCount;

    std::vector<float> windowMaxRms;
    float runningWindowMax = 0.0f;

    for (int step = 0; step < totalSteps; ++step) {
        sim.step(dt);

        const std::size_t n = sim.getParticleCount();
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

    // Every window (not just on average) should stay within a generous
    // bound — a persistent bug would blow this in every late window, not
    // just occasionally.
    const std::size_t backHalfStart = windowMaxRms.size() / 2;
    for (std::size_t i = backHalfStart; i < windowMaxRms.size(); ++i) {
        INFO("window " << i << " (t=[" << i << "," << i + 1 << "]s) peak RMS speed = " << windowMaxRms[i]);
        REQUIRE(windowMaxRms[i] < 10.0f);
    }

    // Trend check: the back half should not be substantially more
    // energetic than the early (post-startup-transient) run — the water
    // bug's signature was a non-decaying, if anything self-reinforcing,
    // pattern. Compare against the peak of seconds 2-4 (past the initial
    // emission ramp-up).
    float earlySettledPeak = *std::max_element(windowMaxRms.begin() + 2, windowMaxRms.begin() + 4);
    float backHalfPeak = *std::max_element(windowMaxRms.begin() + static_cast<long>(backHalfStart), windowMaxRms.end());
    REQUIRE(backHalfPeak <= earlySettledPeak * 2.0f + 1.0f);
}

TEST_CASE("Viscosity produces measurably more local velocity coherence than viscosity=0",
          "[sph_smoke][viscosity]") {
    // Direct empirical check that the kernel-weighted Laplacian viscosity
    // term is doing genuine, measurable coherence work (see
    // averageNeighborVelocityResidual above) — this is the entire source of
    // "genuine SPH character" in a solver that otherwise has no pressure
    // force at all (see SphSmokeSolver's class comment / docs/
    // sph-smoke-sim-api.md's "Pressure/EOS decision" section for why
    // viscosity alone was found sufficient). With viscosity=0 this solver
    // degenerates to ballistic buoyant particles with zero inter-particle
    // coupling; the default viscosity should make each particle's velocity
    // measurably closer to its neighbors' average.
    const float dt = 1.0f / 60.0f;
    const int steps = 60 * 15; // 15 simulated seconds — past initial transients

    SphSmokeSimulation withViscosity; // default params
    for (int i = 0; i < steps; ++i) withViscosity.step(dt);
    const double residualWith = averageNeighborVelocityResidual(withViscosity);

    SphSmokeSimulation withoutViscosity;
    SphSmokeParams p = withoutViscosity.params;
    p.viscosity = 0.0f;
    withoutViscosity.params = p;
    withoutViscosity.reset(fluids::makePlumeScenario());
    for (int i = 0; i < steps; ++i) withoutViscosity.step(dt);
    const double residualWithout = averageNeighborVelocityResidual(withoutViscosity);

    INFO("residual with default viscosity = " << residualWith
                                               << ", residual with viscosity=0 = " << residualWithout);
    REQUIRE(residualWith < residualWithout);
}
