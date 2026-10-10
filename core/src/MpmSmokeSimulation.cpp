#include "fluids/MpmSmokeSimulation.h"

namespace fluids {

MpmSmokeSimulation::MpmSmokeSimulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // MpmSmokeSimulation (e.g. `new Module.MpmSmokeSimulation()` in JS,
    // before any explicit reset() call) already has a live grid/buffer to
    // render — same convention as every other solver. Reuses
    // makePlumeScenario() unchanged, same preset FlipSmokeSimulation uses —
    // no new scenario/preset was needed for this phase (see
    // docs/mpm-smoke-sim-api.md).
    reset(makePlumeScenario());
}

void MpmSmokeSimulation::step(float dt) {
    solver_.step(params, dt);
}

void MpmSmokeSimulation::reset(const SmokeScenario& scenario) {
    solver_.reset(scenario, params);
}

std::size_t MpmSmokeSimulation::getParticleCount() const {
    return solver_.particleCount();
}

const float* MpmSmokeSimulation::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* MpmSmokeSimulation::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

const float* MpmSmokeSimulation::getDensitiesPtr() const {
    return solver_.density().data();
}

const float* MpmSmokeSimulation::getTemperaturesPtr() const {
    return solver_.temperature().data();
}

} // namespace fluids
