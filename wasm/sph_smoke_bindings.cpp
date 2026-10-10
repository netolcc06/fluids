// Embind bindings exposing fluids::SphSmokeSimulation to JavaScript. Kept in
// a separate translation unit from wasm/bindings.cpp (SPH water),
// wasm/flip_bindings.cpp (FLIP water), wasm/mpm_bindings.cpp (MPM water),
// wasm/flip_smoke_bindings.cpp (FLIP/grid smoke), and
// wasm/mpm_smoke_bindings.cpp (MPM/APIC smoke) — all link into the same
// fluids_wasm executable/module and all EMSCRIPTEN_BINDINGS blocks register
// into the same JS Module object, so this is purely a source-organization
// choice, not a separate Module. See docs/sph-smoke-sim-api.md for the full
// JS-facing contract this file implements.
#include <emscripten/bind.h>

#include "fluids/SphSmokeSimulation.h"
#include "fluids/scenario/SmokeScenario.h"
#include "fluids/solvers/sph_smoke/SphSmokeParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Same pattern as every other bindings file: Embind can't return a raw
// pointer as a usable JS value directly, so expose it as an integer byte
// offset into the WASM heap for JS to build a zero-copy Float32Array view
// from.
std::uintptr_t sphSmokeGetPositionsPtr(const SphSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t sphSmokeGetVelocitiesPtr(const SphSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

std::uintptr_t sphSmokeGetDensitiesPtr(const SphSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getDensitiesPtr());
}

std::uintptr_t sphSmokeGetTemperaturesPtr(const SphSmokeSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getTemperaturesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_sph_smoke_module) {
    // NOTE: SmokeScenario is already registered as a value_object by
    // wasm/flip_smoke_bindings.cpp (fluids_flip_smoke_module) — it's shared
    // unchanged across all three smoke solvers (see SmokeScenario.h), so it
    // is NOT re-registered here. value_object registration is process-global
    // per Embind, and registering the same C++ type twice under
    // EMSCRIPTEN_BINDINGS is an error. Likewise makePlumeScenario() is
    // already registered as a free function there and works unmodified for
    // SphSmokeSimulation::reset() too.

    value_object<sph_smoke::SphSmokeParams>("SphSmokeParams")
        .field("buoyancy", &sph_smoke::SphSmokeParams::buoyancy)
        .field("ambientTemperature", &sph_smoke::SphSmokeParams::ambientTemperature)
        .field("sourceTemperature", &sph_smoke::SphSmokeParams::sourceTemperature)
        .field("densityDissipation", &sph_smoke::SphSmokeParams::densityDissipation)
        .field("temperatureDissipation", &sph_smoke::SphSmokeParams::temperatureDissipation)
        .field("smoothingRadius", &sph_smoke::SphSmokeParams::smoothingRadius)
        .field("viscosity", &sph_smoke::SphSmokeParams::viscosity)
        .field("maxParticles", &sph_smoke::SphSmokeParams::maxParticles);

    class_<SphSmokeSimulation>("SphSmokeSimulation")
        .constructor<>()
        .function("step", &SphSmokeSimulation::step)
        .function("reset", &SphSmokeSimulation::reset)
        .function("getParticleCount", &SphSmokeSimulation::getParticleCount)
        .function("getPositionsPtr", &sphSmokeGetPositionsPtr)
        .function("getVelocitiesPtr", &sphSmokeGetVelocitiesPtr)
        .function("getDensitiesPtr", &sphSmokeGetDensitiesPtr)
        .function("getTemperaturesPtr", &sphSmokeGetTemperaturesPtr)
        // Same value_object property gotcha as every other solver's params:
        // reading sim.params returns a fresh plain JS object every time, so
        // `sim.params.buoyancy = x` silently mutates a throwaway copy.
        // Always read-modify-write: `const p = sim.params; p.buoyancy = x;
        // sim.params = p;` (see docs/sph-smoke-sim-api.md).
        .property("params", &SphSmokeSimulation::params);
}
