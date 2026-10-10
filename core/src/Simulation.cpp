#include "fluids/Simulation.h"

namespace fluids {

Simulation::Simulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // Simulation (e.g. `new Module.Simulation()` in JS, before any explicit
    // reset() call) already has particles to render.
    reset(makeDamBreakScenario());
}

void Simulation::step(float dt) {
    solver_.step(params, dt);
}

void Simulation::reset(const Scenario& scenario) {
    solver_.reset(scenario);
}

std::size_t Simulation::getParticleCount() const {
    return solver_.particleCount();
}

const float* Simulation::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* Simulation::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

} // namespace fluids
