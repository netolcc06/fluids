// Embind bindings exposing fluids::Simulation to JavaScript. All
// browser/Emscripten-specific glue lives in this file — core/ stays plain,
// portable C++17. See docs/sim-api.md for the full JS-facing contract this
// file implements.
#include <emscripten/bind.h>

#include "fluids/Simulation.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/sph/SPHParams.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Embind can't return a raw pointer as a usable JS value directly; expose it
// as an integer (byte offset into the WASM heap / Module.HEAPF32.buffer) so
// JS can construct a zero-copy Float32Array view:
//   new Float32Array(Module.HEAPF32.buffer, ptr, count * 2)
std::uintptr_t getPositionsPtr(const Simulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t getVelocitiesPtr(const Simulation& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_module) {
    // Plain-data value types: JS interacts with these as ordinary object
    // literals ({domainWidth: 2, ...}), NOT via `new Module.X()` — value_object
    // registers a JS<->C++ struct converter, not a constructible JS class.
    value_object<sph::SPHParams>("SPHParams")
        .field("gravity", &sph::SPHParams::gravity)
        .field("restDensity", &sph::SPHParams::restDensity)
        .field("gasStiffness", &sph::SPHParams::gasStiffness)
        .field("viscosity", &sph::SPHParams::viscosity)
        .field("smoothingRadius", &sph::SPHParams::smoothingRadius)
        .field("particleMass", &sph::SPHParams::particleMass)
        .field("boundaryDamping", &sph::SPHParams::boundaryDamping);

    value_object<Scenario>("Scenario")
        .field("domainWidth", &Scenario::domainWidth)
        .field("domainHeight", &Scenario::domainHeight)
        .field("spacing", &Scenario::spacing)
        .field("regionX", &Scenario::regionX)
        .field("regionY", &Scenario::regionY)
        .field("regionWidth", &Scenario::regionWidth)
        .field("regionHeight", &Scenario::regionHeight)
        .field("velocityX", &Scenario::velocityX)
        .field("velocityY", &Scenario::velocityY)
        .field("hasRegion2", &Scenario::hasRegion2)
        .field("region2X", &Scenario::region2X)
        .field("region2Y", &Scenario::region2Y)
        .field("region2Width", &Scenario::region2Width)
        .field("region2Height", &Scenario::region2Height)
        .field("region2VelocityX", &Scenario::region2VelocityX)
        .field("region2VelocityY", &Scenario::region2VelocityY);

    // Named preset factory functions, directly callable from JS
    // (Module.makeDamBreakScenario() etc.), each returning a plain Scenario
    // object the frontend can tweak further before calling sim.reset().
    function("makeDamBreakScenario", &makeDamBreakScenario);
    function("makeBlockDropScenario", &makeBlockDropScenario);
    function("makeTwoColumnCollisionScenario", &makeTwoColumnCollisionScenario);

    class_<Simulation>("Simulation")
        .constructor<>()
        .function("step", &Simulation::step)
        .function("reset", &Simulation::reset)
        .function("getParticleCount", &Simulation::getParticleCount)
        .function("getPositionsPtr", &getPositionsPtr)
        .function("getVelocitiesPtr", &getVelocitiesPtr)
        // NOTE: this is a value_object-typed property — reading it returns
        // a fresh plain JS object each time, so `sim.params.viscosity = x`
        // silently mutates a throwaway copy and does NOT persist. Always
        // read-modify-write: `const p = sim.params; p.viscosity = x; sim.params = p;`
        // (see docs/sim-api.md).
        .property("params", &Simulation::params);
}
