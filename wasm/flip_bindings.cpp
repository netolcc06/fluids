// Embind bindings exposing fluids::FlipSimulation to JavaScript. Kept in a
// separate translation unit from wasm/bindings.cpp (the SPH bindings) —
// both link into the same fluids_wasm executable/module and both
// EMSCRIPTEN_BINDINGS blocks register into the same JS Module object, so
// this is purely a source-organization choice, not a separate Module. See
// docs/flip-sim-api.md for the full JS-facing contract this file
// implements.
#include <emscripten/bind.h>

#include "fluids/FlipSimulation.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/flip/FlipParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Same pattern as wasm/bindings.cpp's getPositionsPtr/getVelocitiesPtr:
// Embind can't return a raw pointer as a usable JS value directly, so
// expose it as an integer byte offset into the WASM heap for JS to build a
// zero-copy Float32Array view from (new Float32Array(Module.HEAPF32.buffer,
// ptr, count * 2)).
std::uintptr_t flipGetPositionsPtr(const FlipSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t flipGetVelocitiesPtr(const FlipSimulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_flip_module) {
    // NOTE: Scenario is already registered as a value_object by
    // wasm/bindings.cpp (fluids_module) — it's shared unchanged between SPH
    // and FLIP (see Scenario.h), so it is NOT re-registered here.
    // value_object registration is process-global per Embind, and
    // registering the same C++ type twice under EMSCRIPTEN_BINDINGS is an
    // error.

    value_object<flip::FlipParams>("FlipParams")
        .field("gravity", &flip::FlipParams::gravity)
        .field("flipRatio", &flip::FlipParams::flipRatio)
        .field("cellSize", &flip::FlipParams::cellSize);

    class_<FlipSimulation>("FlipSimulation")
        .constructor<>()
        .function("step", &FlipSimulation::step)
        .function("reset", &FlipSimulation::reset)
        .function("getParticleCount", &FlipSimulation::getParticleCount)
        .function("getPositionsPtr", &flipGetPositionsPtr)
        .function("getVelocitiesPtr", &flipGetVelocitiesPtr)
        // Same value_object property gotcha as Simulation::params: reading
        // sim.params returns a fresh plain JS object every time, so
        // `sim.params.flipRatio = x` silently mutates a throwaway copy.
        // Always read-modify-write: `const p = sim.params; p.flipRatio = x;
        // sim.params = p;` (see docs/flip-sim-api.md).
        .property("params", &FlipSimulation::params);
}
