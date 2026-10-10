#include "fluids/FlipSimulation.h"

namespace fluids {

FlipSimulation::FlipSimulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // FlipSimulation (e.g. `new Module.FlipSimulation()` in JS, before any
    // explicit reset() call) already has particles to render — same as
    // Simulation (SPH).
    reset(makeDamBreakScenario());
}

void FlipSimulation::step(float dt) {
    solver_.step(params, dt);
}

void FlipSimulation::reset(const Scenario& scenario) {
    solver_.reset(scenario, params);
}

std::size_t FlipSimulation::getParticleCount() const {
    return solver_.particleCount();
}

const float* FlipSimulation::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* FlipSimulation::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

} // namespace fluids
