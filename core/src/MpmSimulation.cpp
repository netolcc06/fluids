#include "fluids/MpmSimulation.h"

namespace fluids {

MpmSimulation::MpmSimulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // MpmSimulation (e.g. `new Module.MpmSimulation()` in JS, before any
    // explicit reset() call) already has particles to render — same as
    // Simulation (SPH) and FlipSimulation.
    reset(makeDamBreakScenario());
}

void MpmSimulation::step(float dt) {
    solver_.step(params, dt);
}

void MpmSimulation::reset(const Scenario& scenario) {
    solver_.reset(scenario, params);
}

std::size_t MpmSimulation::getParticleCount() const {
    return solver_.particleCount();
}

const float* MpmSimulation::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* MpmSimulation::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

} // namespace fluids
