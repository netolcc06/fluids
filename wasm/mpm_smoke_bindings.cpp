// Embind bindings exposing fluids::MpmSmokeSimulation to JavaScript. Kept in
// a separate translation unit from wasm/bindings.cpp (SPH), wasm/flip_bindings.cpp
// (FLIP water), wasm/mpm_bindings.cpp (MPM water), and
// wasm/flip_smoke_bindings.cpp (FLIP/grid smoke) — all link into the same
// fluids_wasm executable/module and all EMSCRIPTEN_BINDINGS blocks register
// into the same JS Module object, so this is purely a source-organization
// choice, not a separate Module. See docs/mpm-smoke-sim-api.md for the full
// JS-facing contract this file implements.
#include <emscripten/bind.h>

#include "fluids/MpmSmokeSimulation.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/mpm_smoke/MpmSmokeParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Same pattern as every other bindings file: Embind can't return a raw
// pointer as a usable JS value directly, so expose it as an integer byte
// offset into the WASM heap for JS to build a zero-copy Float32Array view
// from.
std::uintptr_t mpmSmokeGetPositionsPtr(const MpmSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t mpmSmokeGetVelocitiesPtr(const MpmSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

std::uintptr_t mpmSmokeGetDensitiesPtr(const MpmSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getDensitiesPtr());
}

std::uintptr_t mpmSmokeGetTemperaturesPtr(const MpmSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getTemperaturesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_mpm_smoke_module) {
    // NOTE: SmokeScenario is already registered as a value_object by
    // wasm/flip_smoke_bindings.cpp (fluids_flip_smoke_module) — it's shared
    // unchanged between FLIP smoke and MPM smoke (see SmokeScenario.h,
    // reused as-is with no new fields for this phase), so it is NOT
    // re-registered here. value_object registration is process-global per
    // Embind, and registering the same C++ type twice under
    // EMSCRIPTEN_BINDINGS is an error. Likewise makePlumeScenario() is
    // already registered as a free function there and works unmodified for
    // MpmSmokeSimulation::reset() too.

    value_object<mpm_smoke::MpmSmokeParams>("MpmSmokeParams")
        .field("buoyancy", &mpm_smoke::MpmSmokeParams::buoyancy)
        .field("ambientTemperature", &mpm_smoke::MpmSmokeParams::ambientTemperature)
        .field("sourceTemperature", &mpm_smoke::MpmSmokeParams::sourceTemperature)
        .field("densityDissipation", &mpm_smoke::MpmSmokeParams::densityDissipation)
        .field("temperatureDissipation", &mpm_smoke::MpmSmokeParams::temperatureDissipation)
        .field("cellSize", &mpm_smoke::MpmSmokeParams::cellSize)
        .field("damping", &mpm_smoke::MpmSmokeParams::damping)
        .field("maxParticles", &mpm_smoke::MpmSmokeParams::maxParticles);

    class_<MpmSmokeSimulation>("MpmSmokeSimulation")
        .constructor<>()
        .function("step", &MpmSmokeSimulation::step)
        .function("reset", &MpmSmokeSimulation::reset)
        .function("getParticleCount", &MpmSmokeSimulation::getParticleCount)
        .function("getPositionsPtr", &mpmSmokeGetPositionsPtr)
        .function("getVelocitiesPtr", &mpmSmokeGetVelocitiesPtr)
        .function("getDensitiesPtr", &mpmSmokeGetDensitiesPtr)
        .function("getTemperaturesPtr", &mpmSmokeGetTemperaturesPtr)
        // Same value_object property gotcha as every other solver's params:
        // reading sim.params returns a fresh plain JS object every time, so
        // `sim.params.buoyancy = x` silently mutates a throwaway copy.
        // Always read-modify-write: `const p = sim.params; p.buoyancy = x;
        // sim.params = p;` (see docs/mpm-smoke-sim-api.md).
        .property("params", &MpmSmokeSimulation::params);
}
