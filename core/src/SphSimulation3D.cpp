#include "fluids/SphSimulation3D.h"

namespace fluids {

SphSimulation3D::SphSimulation3D() {
    // Start from a sensible default scenario so a freshly-constructed
    // SphSimulation3D (e.g. `new Module.SphSimulation3D()` in JS, before any
    // explicit reset() call) already has particles to render — same as the
    // 2D Simulation's default-construction behavior.
    reset(makeDamBreak3DScenario());
}

void SphSimulation3D::step(float dt) {
    solver_.step(params, dt);
}

void SphSimulation3D::reset(const Scenario3D& scenario) {
    solver_.reset(scenario);
}

std::size_t SphSimulation3D::getParticleCount() const {
    return solver_.particleCount();
}

const float* SphSimulation3D::getPositionsPtr() const {
    return reinterpret_cast<const float*>(solver_.positions().data());
}

const float* SphSimulation3D::getVelocitiesPtr() const {
    return reinterpret_cast<const float*>(solver_.velocities().data());
}

} // namespace fluids
