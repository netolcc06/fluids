#pragma once

namespace fluids {

// Describes the initial state handed to SphSimulation3D::reset(): domain
// bounds plus up to two axis-aligned box particle-seed regions. The 3D
// sibling of Scenario (see Scenario.h) — a new, independent struct rather
// than an extension of the 2D one, since a box region genuinely has more
// fields (width/height/depth, vx/vy/vz) than a 2D rectangle.
//
// Coordinate convention (applies to every 3D solver, not just SPH): origin
// (0, 0, 0) at one corner of the domain box, +x right, +y up (same as the
// 2D convention), +z toward the viewer (out of the screen), forming a
// right-handed system (x cross y = z). The simulation box is
// [0, domainWidth] x [0, domainHeight] x [0, domainDepth]. This matters a
// lot for whoever wires up the 3D camera next: a standard right-handed
// camera looking down -z at the origin sees +x to the right and +y up
// exactly like the 2D renderer's screen, with the box extending toward the
// camera along +z.
//
// This is intentionally a flat, plain-old-data struct (no nested
// vectors/containers) so it binds as a trivial Embind value_object, same
// reasoning as the 2D Scenario. Two flat box regions (mirroring the 2D
// Scenario's hasRegion2 pattern) is enough to express every preset here
// (Dam Break / Block Drop = 1 region, Two-Column Collision = 2 regions
// converging) plus arbitrary two-body custom setups from JS.
struct Scenario3D {
    // Domain size, meters. The simulation box is [0, domainWidth] x
    // [0, domainHeight] x [0, domainDepth].
    float domainWidth = 1.2f;
    float domainHeight = 1.0f;
    float domainDepth = 0.6f;

    // Particle grid spacing used when seeding both regions, meters.
    float spacing = 0.06f;

    // Primary seed region: an axis-aligned box
    // [regionX, regionX+regionWidth] x [regionY, regionY+regionHeight] x
    // [regionZ, regionZ+regionDepth], filled with a regular 3D grid of
    // particles at `spacing`, each given initial velocity
    // (velocityX, velocityY, velocityZ) in m/s.
    float regionX = 0.05f;
    float regionY = 0.05f;
    float regionZ = 0.05f;
    float regionWidth = 0.5f;
    float regionHeight = 0.8f;
    float regionDepth = 0.5f;
    float velocityX = 0.0f;
    float velocityY = 0.0f;
    float velocityZ = 0.0f;

    // Optional second seed region, same shape as the primary one. Ignored
    // entirely when hasRegion2 is false. Used by the Two-Column Collision
    // preset (two blocks with opposing velocityX); the frontend can also set
    // this directly for custom two-body setups without new C++.
    bool hasRegion2 = false;
    float region2X = 0.0f;
    float region2Y = 0.0f;
    float region2Z = 0.0f;
    float region2Width = 0.0f;
    float region2Height = 0.0f;
    float region2Depth = 0.0f;
    float region2VelocityX = 0.0f;
    float region2VelocityY = 0.0f;
    float region2VelocityZ = 0.0f;
};

// Named presets. Convenience constructors only — plain factory functions
// that return a Scenario3D value with sensible defaults. No solver logic is
// specialized on which preset was used; the solver only ever sees the
// resulting Scenario3D's generic fields.
Scenario3D makeDamBreak3DScenario();
Scenario3D makeBlockDrop3DScenario();
Scenario3D makeTwoColumnCollision3DScenario();

} // namespace fluids
