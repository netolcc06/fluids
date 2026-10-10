#include "fluids/FlipSmokeSimulation.h"

namespace fluids {

FlipSmokeSimulation::FlipSmokeSimulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // FlipSmokeSimulation (e.g. `new Module.FlipSmokeSimulation()` in JS,
    // before any explicit reset() call) already has a live grid to render —
    // same convention as Simulation (SPH) and FlipSimulation (FLIP water).
    reset(makePlumeScenario());
}

void FlipSmokeSimulation::step(float dt) {
    solver_.step(params, dt);
}

void FlipSmokeSimulation::reset(const SmokeScenario& scenario) {
    solver_.reset(scenario, params);
}

int FlipSmokeSimulation::getGridWidth() const {
    return solver_.gridWidth();
}

int FlipSmokeSimulation::getGridHeight() const {
    return solver_.gridHeight();
}

const float* FlipSmokeSimulation::getDensityFieldPtr() const {
    return solver_.density().data();
}

const float* FlipSmokeSimulation::getTemperatureFieldPtr() const {
    return solver_.temperature().data();
}

} // namespace fluids
