#pragma once

namespace fluids {

// Describes the initial/ongoing state handed to FlipSmokeSimulation::reset():
// domain bounds plus a single rectangular source region that CONTINUOUSLY
// emits density/temperature every step (see
// fluids::smoke::FlipSmokeSolver::emitSource). This is a deliberately
// separate struct from fluids::Scenario (the water preset shape), not an
// overload of it — the two describe conceptually different things: water's
// Scenario seeds a one-shot particle region with an initial velocity and
// then the particles are on their own; smoke has no particles at all and
// its source region stays "live," injecting more density/temperature into
// the same cells on every single step for as long as the simulation runs.
// Reusing Scenario's regionX/Y/Width/Height/velocityX/Y fields for this
// would be misleading (there's no "velocity" to seed, and the region never
// stops mattering the way a water seed region does after the first step).
//
// Same coordinate convention as fluids::Scenario: origin (0, 0) at the
// domain's bottom-left corner, +x right, +y up, all distances in meters.
//
// Flat POD struct for the same reason as Scenario: trivial Embind
// value_object, constructible as a plain JS object literal.
struct SmokeScenario {
    // Domain size, meters. The simulation box is [0, domainWidth] x
    // [0, domainHeight].
    float domainWidth = 1.2f;
    float domainHeight = 2.0f;

    // Source region: an axis-aligned rectangle
    // [sourceX, sourceX+sourceWidth] x [sourceY, sourceY+sourceHeight].
    // Every step, every grid cell whose center falls inside this rectangle
    // has density added and temperature blended toward
    // FlipSmokeParams::sourceTemperature (see
    // fluids::smoke::FlipSmokeSolver::emitSource) — think of it as a
    // chimney/plume base or a candle flame footprint, not a one-shot seed.
    float sourceX = 0.45f;
    float sourceY = 0.0f;
    float sourceWidth = 0.3f;
    float sourceHeight = 0.15f;

    // Density units added per second within the source region, before
    // per-cell clamping to [0, 1] (see FlipSmokeSolver::emitSource).
    // `sourceTemperature` — the OTHER emission parameter — deliberately
    // lives on FlipSmokeParams instead of here: it is a live-tunable
    // physical constant (how hot the source is), not a shape/placement
    // property of the scenario, matching the split already established by
    // FlipParams::cellSize living outside water's Scenario. `emissionRate`
    // stays here because how fast a *particular* preset pumps out smoke is
    // more a scenario characteristic (a wide slow vent vs. a small fast
    // jet) than a global tunable.
    float emissionRate = 2.0f;
};

// Named preset. A modest source region near the bottom-center of a tall-ish
// domain (2.0m tall vs. 1.2m wide) so the plume has room to rise and curl
// before hitting the top Solid boundary ring.
SmokeScenario makePlumeScenario();

} // namespace fluids
