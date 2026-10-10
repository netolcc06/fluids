#include "fluids/SphSmokeSimulation.h"

namespace fluids {

SphSmokeSimulation::SphSmokeSimulation() {
    // Start from a sensible default scenario so a freshly-constructed
    // SphSmokeSimulation already has a live buffer to render — same
    // convention as every other solver. Reuses makePlumeScenario()
    // unchanged, same preset the other two smoke solvers use — no new
    // scenario/preset was needed for this phase.
    reset(makePlumeScenario());
}

void SphSmokeSimulation::step(float dt) {
    solver_.step(params, dt);
}

void SphSmokeSimulation::reset(const SmokeScenario& scenario) {
    solver_.reset(scenario, params);
}

std::size_t SphSmokeSimulation::getParticleCount() const {
    return solver_.particleCount();
}

const float* SphSmokeSimulation::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* SphSmokeSimulation::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

const float* SphSmokeSimulation::getDensitiesPtr() const {
    return solver_.density().data();
}

const float* SphSmokeSimulation::getTemperaturesPtr() const {
    return solver_.temperature().data();
}

} // namespace fluids
