// Embind bindings exposing fluids::MpmSimulation to JavaScript. Kept in a
// separate translation unit from wasm/bindings.cpp (SPH) and
// wasm/flip_bindings.cpp (FLIP) — all three link into the same fluids_wasm
// executable/module and all three EMSCRIPTEN_BINDINGS blocks register into
// the same JS Module object, so this is purely a source-organization
// choice, not a separate Module. See docs/mpm-sim-api.md for the full
// JS-facing contract this file implements.
#include <emscripten/bind.h>

#include "fluids/MpmSimulation.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/mpm/MpmParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Same pattern as wasm/bindings.cpp's getPositionsPtr/getVelocitiesPtr:
// Embind can't return a raw pointer as a usable JS value directly, so
// expose it as an integer byte offset into the WASM heap for JS to build a
// zero-copy Float32Array view from (new Float32Array(Module.HEAPF32.buffer,
// ptr, count * 2)).
std::uintptr_t mpmGetPositionsPtr(const MpmSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t mpmGetVelocitiesPtr(const MpmSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_mpm_module) {
    // NOTE: Scenario is already registered as a value_object by
    // wasm/bindings.cpp (fluids_module) — it's shared unchanged across SPH,
    // FLIP, and MPM (see Scenario.h), so it is NOT re-registered here.
    // value_object registration is process-global per Embind, and
    // registering the same C++ type twice under EMSCRIPTEN_BINDINGS is an
    // error.

    value_object<mpm::MpmParams>("MpmParams")
        .field("gravity", &mpm::MpmParams::gravity)
        .field("restDensity", &mpm::MpmParams::restDensity)
        .field("stiffness", &mpm::MpmParams::stiffness)
        .field("viscosity", &mpm::MpmParams::viscosity)
        .field("cellSize", &mpm::MpmParams::cellSize);

    class_<MpmSimulation>("MpmSimulation")
        .constructor<>()
        .function("step", &MpmSimulation::step)
        .function("reset", &MpmSimulation::reset)
        .function("getParticleCount", &MpmSimulation::getParticleCount)
        .function("getPositionsPtr", &mpmGetPositionsPtr)
        .function("getVelocitiesPtr", &mpmGetVelocitiesPtr)
        // Same value_object property gotcha as Simulation::params /
        // FlipSimulation::params: reading sim.params returns a fresh plain
        // JS object every time, so `sim.params.stiffness = x` silently
        // mutates a throwaway copy. Always read-modify-write: `const p =
        // sim.params; p.stiffness = x; sim.params = p;` (see
        // docs/mpm-sim-api.md).
        .property("params", &MpmSimulation::params);
}
