#pragma once

namespace fluids {

// Describes the initial state handed to Simulation::reset(): domain bounds
// plus up to two rectangular particle-seed regions. Coordinate convention
// (applies everywhere in core/): origin (0, 0) at the domain's bottom-left
// corner, +x to the right, +y up. All distances are in meters.
//
// This is intentionally a flat, plain-old-data struct (no nested
// vectors/containers) so it binds as a trivial Embind value_object — the
// frontend constructs one as a plain JS object literal, no marshalling
// surprises. Two flat regions (rather than the master plan's more open-ended
// "list of seeded regions") is a deliberate scope trade-off: it's enough to
// express every Phase 1 preset (Dam Break / Block Drop = 1 region,
// Two-Column Collision = 2 regions converging) plus arbitrary two-body
// custom setups from JS, while keeping the JS-facing contract dead simple.
struct Scenario {
    // Domain size, meters. The simulation box is [0, domainWidth] x
    // [0, domainHeight].
    float domainWidth = 2.0f;
    float domainHeight = 1.5f;

    // Particle grid spacing used when seeding both regions, meters.
    float spacing = 0.035f;

    // Primary seed region: an axis-aligned rectangle
    // [regionX, regionX+regionWidth] x [regionY, regionY+regionHeight],
    // filled with a regular grid of particles at `spacing`, each given
    // initial velocity (velocityX, velocityY) in m/s.
    float regionX = 0.1f;
    float regionY = 0.1f;
    float regionWidth = 0.6f;
    float regionHeight = 0.8f;
    float velocityX = 0.0f;
    float velocityY = 0.0f;

    // Optional second seed region, same shape as the primary one. Ignored
    // entirely when hasRegion2 is false. Used by the Two-Column Collision
    // preset (two blocks with opposing velocityX); the frontend can also
    // set this directly for custom two-body setups without new C++.
    bool hasRegion2 = false;
    float region2X = 0.0f;
    float region2Y = 0.0f;
    float region2Width = 0.0f;
    float region2Height = 0.0f;
    float region2VelocityX = 0.0f;
    float region2VelocityY = 0.0f;
};

// Named presets. These are convenience constructors only — plain factory
// functions that return a Scenario value with sensible defaults. No solver
// logic is specialized on which preset was used; the solver only ever sees
// the resulting Scenario's generic fields.
Scenario makeDamBreakScenario();
Scenario makeBlockDropScenario();
Scenario makeTwoColumnCollisionScenario();

} // namespace fluids
