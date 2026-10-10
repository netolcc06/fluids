#include <algorithm>
#include <cmath>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>

#include "fluids/FlipSmokeSimulation.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/smoke/FlipSmokeParams.h"
#include "fluids/solvers/smoke/FlipSmokeSolver.h"

using fluids::FlipSmokeSimulation;
using fluids::SmokeScenario;
using fluids::flip::CellType;
using fluids::smoke::FlipSmokeParams;
using fluids::smoke::FlipSmokeSolver;

namespace {

bool allFinite(const float* buf, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(buf[i])) return false;
    }
    return true;
}

double totalDensity(const FlipSmokeSolver& solver) {
    double sum = 0.0;
    for (float d : solver.density()) sum += d;
    return sum;
}

} // namespace

TEST_CASE("FlipSmokeSolver reset allocates grid at expected dimensions", "[smoke][construction]") {
    FlipSmokeSolver solver;
    FlipSmokeParams params;
    params.cellSize = 0.1f;

    SmokeScenario scenario;
    scenario.domainWidth = 1.0f;
    scenario.domainHeight = 2.0f;

    solver.reset(scenario, params);

    REQUIRE(solver.gridWidth() == 10);
    REQUIRE(solver.gridHeight() == 20);
    REQUIRE(solver.density().size() == static_cast<std::size_t>(10 * 20));
    REQUIRE(solver.temperature().size() == static_cast<std::size_t>(10 * 20));

    // Boundary ring is Solid, interior is Fluid — no Air state exists for
    // smoke (see FlipSmokeSolver::classifyCells).
    const auto& grid = solver.grid();
    REQUIRE(grid.cell(0, 0) == CellType::Solid);
    REQUIRE(grid.cell(9, 19) == CellType::Solid);
    REQUIRE(grid.cell(5, 10) == CellType::Fluid);

    // Freshly reset: no density yet, temperature at ambient everywhere.
    for (float d : solver.density()) REQUIRE(d == 0.0f);
    for (float t : solver.temperature()) REQUIRE(t == params.ambientTemperature);
}

TEST_CASE("FlipSmokeSimulation default construction seeds a live grid", "[smoke][construction]") {
    FlipSmokeSimulation sim;
    REQUIRE(sim.getGridWidth() > 0);
    REQUIRE(sim.getGridHeight() > 0);
    REQUIRE(sim.getDensityFieldPtr() != nullptr);
    REQUIRE(sim.getTemperatureFieldPtr() != nullptr);
}

TEST_CASE("Buoyancy drives net upward velocity above a hot source, more than elsewhere",
          "[smoke][buoyancy]") {
    FlipSmokeSolver solver;
    FlipSmokeParams params;
    params.buoyancy = 6.0f;
    params.ambientTemperature = 0.0f;
    params.sourceTemperature = 1.0f;
    params.densityDissipation = 0.1f;
    params.temperatureDissipation = 0.0f; // isolate buoyancy from temperature relaxing away this run
    params.cellSize = 0.05f;

    SmokeScenario scenario;
    scenario.domainWidth = 1.0f;
    scenario.domainHeight = 1.5f;
    scenario.sourceX = 0.4f;
    scenario.sourceWidth = 0.2f;
    scenario.sourceY = 0.0f;
    scenario.sourceHeight = 0.1f;
    scenario.emissionRate = 5.0f;

    solver.reset(scenario, params);

    const float dt = 1.0f / 120.0f;
    for (int i = 0; i < 30; ++i) {
        solver.step(params, dt);
    }

    const auto& grid = solver.grid();
    const int nx = grid.nx;
    const int ny = grid.ny;
    const float h = grid.h;

    // A row of v-faces well above the source but still comfortably inside
    // the domain (away from the top Solid ring).
    const int bandJ = std::clamp(static_cast<int>(0.5f / h), 1, ny - 1);
    REQUIRE(bandJ >= 1);
    REQUIRE(bandJ <= ny - 1);

    // Column range directly above the source rectangle.
    const int iSrc0 = std::clamp(static_cast<int>(scenario.sourceX / h), 0, nx - 1);
    const int iSrc1 = std::clamp(static_cast<int>((scenario.sourceX + scenario.sourceWidth) / h), 0, nx - 1);
    REQUIRE(iSrc1 >= iSrc0);

    double aboveSourceSum = 0.0;
    int aboveSourceCount = 0;
    double elsewhereSum = 0.0;
    int elsewhereCount = 0;
    for (int i = 0; i < nx; ++i) {
        const float v = grid.V(i, bandJ);
        if (i >= iSrc0 && i <= iSrc1) {
            aboveSourceSum += v;
            ++aboveSourceCount;
        } else {
            elsewhereSum += v;
            ++elsewhereCount;
        }
    }
    REQUIRE(aboveSourceCount > 0);
    REQUIRE(elsewhereCount > 0);

    const double aboveSourceAvg = aboveSourceSum / aboveSourceCount;
    const double elsewhereAvg = elsewhereSum / elsewhereCount;

    // Buoyancy should have pushed a clear net upward velocity directly
    // above the hot source, and that should be higher than the (closed-box
    // return-flow-dominated) average everywhere else at the same height.
    REQUIRE(aboveSourceAvg > 0.0);
    REQUIRE(aboveSourceAvg > elsewhereAvg);
}

TEST_CASE("Emission increases total density, and dissipation keeps it bounded rather than unbounded",
          "[smoke][dissipation]") {
    // Buoyancy off and a closed box with zero initial velocity everywhere
    // means velocity never becomes nonzero (pressure projection of an
    // all-zero divergence-free field is a no-op, and semi-Lagrangian
    // advection through a zero velocity field is the identity) — so this
    // isolates the emit+dissipate recurrence
    // `d(n+1) = (d(n) + emissionRate*dt) * (1 - densityDissipation*dt)`
    // pointwise in the source cells, with no advection/projection
    // interference. This is the "checking the dissipation term in
    // isolation" alternative the task calls out, while still running the
    // real FlipSmokeSolver::step() pipeline end to end (not reimplementing
    // the formula in the test).
    FlipSmokeSolver solver;
    FlipSmokeParams params;
    params.buoyancy = 0.0f;
    params.ambientTemperature = 0.0f;
    params.sourceTemperature = 1.0f;
    // densityDissipation chosen large (a fast ~1/2=0.5s time constant) so
    // the recurrence visibly converges within the step budget below rather
    // than still slowly crawling toward equilibrium.
    params.densityDissipation = 2.0f;
    params.temperatureDissipation = 0.0f;
    params.cellSize = 0.1f;

    SmokeScenario scenario;
    scenario.domainWidth = 1.0f;
    scenario.domainHeight = 1.0f;
    scenario.sourceX = 0.3f;
    scenario.sourceY = 0.3f;
    scenario.sourceWidth = 0.4f;
    scenario.sourceHeight = 0.4f;
    scenario.emissionRate = 0.2f; // deliberately small vs. densityDissipation, see equilibrium note above

    solver.reset(scenario, params);
    REQUIRE(totalDensity(solver) == 0.0);

    const float dt = 1.0f / 120.0f;

    // Increases initially.
    for (int i = 0; i < 5; ++i) solver.step(params, dt);
    const double earlySum = totalDensity(solver);
    REQUIRE(earlySum > 0.0);

    for (int i = 0; i < 100; ++i) solver.step(params, dt);
    const double midSum = totalDensity(solver);
    REQUIRE(midSum > earlySum);

    // Continues running to near steady state (emission and dissipation
    // balance out) rather than growing without bound.
    for (int i = 0; i < 400; ++i) solver.step(params, dt);
    const double lateSum = totalDensity(solver);

    for (int i = 0; i < 400; ++i) solver.step(params, dt);
    const double laterSum = totalDensity(solver);

    // Approximate per-cell equilibrium (small-dt limit):
    // d* ~= emissionRate / densityDissipation, well under the 1.0 clamp for
    // these params (0.2/0.5 = 0.4) — sanity bound so the test would catch a
    // dissipation term that silently does nothing (which would instead
    // saturate every source cell at the 1.0 clamp).
    const double approxEqPerCell = static_cast<double>(scenario.emissionRate) / static_cast<double>(params.densityDissipation);
    const double sourceCellCount = static_cast<double>(solver.gridWidth()) * static_cast<double>(solver.gridHeight());
    REQUIRE(laterSum < approxEqPerCell * sourceCellCount); // well under the "always saturate to 1.0" upper bound
    REQUIRE(laterSum <= sourceCellCount * 1.0);             // hard clamp bound, never exceeded

    // Converged: growth has essentially stopped between the two late
    // windows, i.e. dissipation is actively balancing ongoing emission
    // rather than density growing without bound.
    REQUIRE(std::abs(laterSum - lateSum) < 0.05 * lateSum);

    // Never negative anywhere, at every step (checked continuously above via
    // totalDensity implicitly summing non-negative contributions is not a
    // per-cell guarantee, so check explicitly here on the final field).
    for (float d : solver.density()) {
        REQUIRE(d >= 0.0f);
    }
}

TEST_CASE("Plume preset stability: no NaN/Inf, values stay in a sane range, over several seconds",
          "[smoke][stability]") {
    FlipSmokeSimulation sim;
    SmokeScenario scenario = fluids::makePlumeScenario();
    sim.reset(scenario);

    const int w = sim.getGridWidth();
    const int h = sim.getGridHeight();
    REQUIRE(w > 0);
    REQUIRE(h > 0);
    const std::size_t cellCount = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    const float dt = 1.0f / 120.0f;
    const int steps = 120 * 5; // 5 simulated seconds

    for (int i = 0; i < steps; ++i) {
        sim.step(dt);

        const float* density = sim.getDensityFieldPtr();
        const float* temperature = sim.getTemperatureFieldPtr();
        REQUIRE(allFinite(density, cellCount));
        REQUIRE(allFinite(temperature, cellCount));

        for (std::size_t k = 0; k < cellCount; ++k) {
            REQUIRE(density[k] >= -1e-4f);
            REQUIRE(density[k] <= 1.0f + 1e-4f);
        }
    }

    // Manual-trace-style sanity check: the bulk of the density should have
    // risen above the source region by now (a density-weighted centroid
    // height), not be stuck at the source or have vanished entirely.
    const float* density = sim.getDensityFieldPtr();
    const float cellSize = sim.params.cellSize;
    double weightedY = 0.0;
    double totalD = 0.0;
    for (int j = 0; j < h; ++j) {
        const float cy = (static_cast<float>(j) + 0.5f) * cellSize;
        for (int i = 0; i < w; ++i) {
            const float d = density[static_cast<std::size_t>(j) * w + i];
            weightedY += static_cast<double>(d) * cy;
            totalD += d;
        }
    }
    REQUIRE(totalD > 0.0); // smoke hasn't fully dissipated away
    const double centroidY = weightedY / totalD;
    REQUIRE(centroidY > scenario.sourceY + scenario.sourceHeight); // risen above the source strip
}
