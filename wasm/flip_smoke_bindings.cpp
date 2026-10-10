// Embind bindings exposing fluids::FlipSmokeSimulation to JavaScript. Kept
// in a separate translation unit from wasm/bindings.cpp (SPH) and
// wasm/flip_bindings.cpp (FLIP water) — all three link into the same
// fluids_wasm executable/module and all EMSCRIPTEN_BINDINGS blocks register
// into the same JS Module object, so this is purely a source-organization
// choice, not a separate Module. See docs/flip-smoke-sim-api.md for the
// full JS-facing contract this file implements.
#include <emscripten/bind.h>

#include "fluids/FlipSmokeSimulation.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/smoke/FlipSmokeParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Same pattern as wasm/bindings.cpp's getPositionsPtr/getVelocitiesPtr and
// wasm/flip_bindings.cpp's flipGetPositionsPtr/flipGetVelocitiesPtr:
// Embind can't return a raw pointer as a usable JS value directly, so
// expose it as an integer byte offset into the WASM heap for JS to build a
// zero-copy Float32Array view from (new Float32Array(Module.HEAPF32.buffer,
// ptr, width * height)).
std::uintptr_t smokeGetDensityFieldPtr(const FlipSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getDensityFieldPtr());
}

std::uintptr_t smokeGetTemperatureFieldPtr(const FlipSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getTemperatureFieldPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_flip_smoke_module) {
    value_object<smoke::FlipSmokeParams>("FlipSmokeParams")
        .field("buoyancy", &smoke::FlipSmokeParams::buoyancy)
        .field("ambientTemperature", &smoke::FlipSmokeParams::ambientTemperature)
        .field("sourceTemperature", &smoke::FlipSmokeParams::sourceTemperature)
        .field("densityDissipation", &smoke::FlipSmokeParams::densityDissipation)
        .field("temperatureDissipation", &smoke::FlipSmokeParams::temperatureDissipation)
        .field("cellSize", &smoke::FlipSmokeParams::cellSize);

    value_object<SmokeScenario>("SmokeScenario")
        .field("domainWidth", &SmokeScenario::domainWidth)
        .field("domainHeight", &SmokeScenario::domainHeight)
        .field("sourceX", &SmokeScenario::sourceX)
        .field("sourceY", &SmokeScenario::sourceY)
        .field("sourceWidth", &SmokeScenario::sourceWidth)
        .field("sourceHeight", &SmokeScenario::sourceHeight)
        .field("emissionRate", &SmokeScenario::emissionRate);

    // Named preset factory, directly callable from JS
    // (Module.makePlumeScenario()), returning a plain SmokeScenario object
    // the frontend can tweak further before calling sim.reset().
    function("makePlumeScenario", &makePlumeScenario);

    class_<FlipSmokeSimulation>("FlipSmokeSimulation")
        .constructor<>()
        .function("step", &FlipSmokeSimulation::step)
        .function("reset", &FlipSmokeSimulation::reset)
        .function("getGridWidth", &FlipSmokeSimulation::getGridWidth)
        .function("getGridHeight", &FlipSmokeSimulation::getGridHeight)
        .function("getDensityFieldPtr", &smokeGetDensityFieldPtr)
        .function("getTemperatureFieldPtr", &smokeGetTemperatureFieldPtr)
        // Same value_object property gotcha as Simulation::params /
        // FlipSimulation::params: reading sim.params returns a fresh plain
        // JS object every time, so `sim.params.buoyancy = x` silently
        // mutates a throwaway copy. Always read-modify-write:
        // `const p = sim.params; p.buoyancy = x; sim.params = p;` (see
        // docs/flip-smoke-sim-api.md).
        .property("params", &FlipSmokeSimulation::params);
}
