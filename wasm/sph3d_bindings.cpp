// Embind bindings exposing fluids::SphSimulation3D to JavaScript — the 3D
// sibling of wasm/bindings.cpp (2D SPH water). All browser/Emscripten-
// specific glue lives in this file — core/ stays plain, portable C++17. See
// docs/sph3d-sim-api.md for the full JS-facing contract this file
// implements, including the count*3 (not count*2) buffer layout.
#include <emscripten/bind.h>

#include "fluids/SphSimulation3D.h"
#include "fluids/scenario/Scenario3D.h"
#include "fluids/solvers/sph3d/SphParams3D.h"

using namespace emscripten;
using namespace fluids;

namespace {

// Embind can't return a raw pointer as a usable JS value directly; expose it
// as an integer (byte offset into the WASM heap / Module.HEAPF32.buffer) so
// JS can construct a zero-copy Float32Array view:
//   new Float32Array(Module.HEAPF32.buffer, ptr, count * 3)
// (note: *3, not *2 — three floats per particle in 3D).
std::uintptr_t getPositionsPtr3D(const SphSimulation3D& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getPositionsPtr());
}

std::uintptr_t getVelocitiesPtr3D(const SphSimulation3D& sim) {
    return reinterpret_cast<std::uintptr_t>(sim.getVelocitiesPtr());
}

} // namespace

EMSCRIPTEN_BINDINGS(fluids_sph3d_module) {
    // Plain-data value types: JS interacts with these as ordinary object
    // literals ({domainWidth: 2, ...}), NOT via `new Module.X()` — same
    // value_object convention as every other binding file in this project.
    value_object<sph3d::SphParams3D>("SphParams3D")
        .field("gravity", &sph3d::SphParams3D::gravity)
        .field("restDensity", &sph3d::SphParams3D::restDensity)
        .field("gasStiffness", &sph3d::SphParams3D::gasStiffness)
        .field("viscosity", &sph3d::SphParams3D::viscosity)
        .field("smoothingRadius", &sph3d::SphParams3D::smoothingRadius)
        .field("particleMass", &sph3d::SphParams3D::particleMass)
        .field("boundaryDamping", &sph3d::SphParams3D::boundaryDamping);

    value_object<Scenario3D>("Scenario3D")
        .field("domainWidth", &Scenario3D::domainWidth)
        .field("domainHeight", &Scenario3D::domainHeight)
        .field("domainDepth", &Scenario3D::domainDepth)
        .field("spacing", &Scenario3D::spacing)
        .field("regionX", &Scenario3D::regionX)
        .field("regionY", &Scenario3D::regionY)
        .field("regionZ", &Scenario3D::regionZ)
        .field("regionWidth", &Scenario3D::regionWidth)
        .field("regionHeight", &Scenario3D::regionHeight)
        .field("regionDepth", &Scenario3D::regionDepth)
        .field("velocityX", &Scenario3D::velocityX)
        .field("velocityY", &Scenario3D::velocityY)
        .field("velocityZ", &Scenario3D::velocityZ)
        .field("hasRegion2", &Scenario3D::hasRegion2)
        .field("region2X", &Scenario3D::region2X)
        .field("region2Y", &Scenario3D::region2Y)
        .field("region2Z", &Scenario3D::region2Z)
        .field("region2Width", &Scenario3D::region2Width)
        .field("region2Height", &Scenario3D::region2Height)
        .field("region2Depth", &Scenario3D::region2Depth)
        .field("region2VelocityX", &Scenario3D::region2VelocityX)
        .field("region2VelocityY", &Scenario3D::region2VelocityY)
        .field("region2VelocityZ", &Scenario3D::region2VelocityZ);

    // Named preset factory functions, directly callable from JS
    // (Module.makeDamBreak3DScenario() etc.), each returning a plain
    // Scenario3D object the frontend can tweak further before calling
    // sim.reset().
    function("makeDamBreak3DScenario", &makeDamBreak3DScenario);
    function("makeBlockDrop3DScenario", &makeBlockDrop3DScenario);
    function("makeTwoColumnCollision3DScenario", &makeTwoColumnCollision3DScenario);

    class_<SphSimulation3D>("SphSimulation3D")
        .constructor<>()
        .function("step", &SphSimulation3D::step)
        .function("reset", &SphSimulation3D::reset)
        .function("getParticleCount", &SphSimulation3D::getParticleCount)
        .function("getPositionsPtr", &getPositionsPtr3D)
        .function("getVelocitiesPtr", &getVelocitiesPtr3D)
        // NOTE: this is a value_object-typed property — reading it returns
        // a fresh plain JS object each time, so `sim.params.viscosity = x`
        // silently mutates a throwaway copy and does NOT persist. Always
        // read-modify-write: `const p = sim.params; p.viscosity = x; sim.params = p;`
        // (verified against this build the same way the 2D contract was —
        // see docs/sph3d-sim-api.md).
        .property("params", &SphSimulation3D::params);
}
