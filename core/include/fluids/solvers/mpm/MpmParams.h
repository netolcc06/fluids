#pragma once

namespace fluids::mpm {

// Live-tunable MLS-MPM parameters. Mirrors sph::SPHParams' naming where the
// concept is analogous (gravity, restDensity, an EOS stiffness, viscosity),
// and deliberately omits fields that don't apply to MPM (smoothingRadius,
// boundaryDamping — MPM's grid transfer already implies a fixed stencil
// width and its boundary handling is a margin-based grid-velocity clamp,
// not a per-particle bounce, see MpmSolver).
struct MpmParams {
    // Gravitational acceleration applied to grid velocities along +y each
    // step, m/s^2. Same convention as SPHParams::gravity/FlipParams::gravity:
    // domain +y is up, so the default -9.81 pulls material down.
    float gravity = -9.81f;

    // Rest ("target") density, kg/m^2 (2D areal density, same convention as
    // SPHParams::restDensity — not volumetric). Used both to calibrate
    // per-particle mass at reset() (see the particleMass note below) and as
    // the reference density in the weakly-compressible EOS pressure(J) =
    // stiffness * ((1/J)^gamma - 1), since 1/J == density/restDensity for a
    // particle whose deformation gradient determinant is J.
    float restDensity = 1000.0f;

    // Weakly-compressible EOS stiffness constant (this solver's analog of
    // SPHParams::gasStiffness). Higher = less compressible / stiffer water,
    // but needs a smaller stable substep — see docs/mpm-sim-api.md for the
    // empirically-verified default/substep pairing. The EOS exponent
    // (gamma = 7, matching SPH's Tait exponent) is a fixed internal
    // constant, not exposed, per the same reasoning SPH doesn't expose it.
    float stiffness = 20.0f;

    // Optional viscous stress coefficient: adds viscosity * (C + C^T) to the
    // per-particle stress tensor, where C is the APIC affine velocity matrix
    // (a cheap local velocity-gradient estimate already computed every
    // step). 0 (the default) disables this term entirely — pressure-only
    // stress is a perfectly acceptable Phase 3 baseline, and the built-in
    // presets/tests all run with this at 0. Positive values smooth out
    // relative velocity between nearby material the same way
    // SPHParams::viscosity does, but push the stable substep down further
    // if increased a lot — tune with a smaller substep if you raise this.
    float viscosity = 0.0f;

    // Node-centered grid spacing (dx), meters. Grid resolution is derived
    // from Scenario::domainWidth/domainHeight and this value, and is only
    // (re)allocated on reset() — changing cellSize without calling reset()
    // afterward has no effect until the next reset() (same pattern as
    // FlipParams::cellSize). Cost scales roughly ~1/cellSize^2 for a fixed
    // domain (see docs/mpm-sim-api.md's performance section).
    float cellSize = 0.02f;
};

} // namespace fluids::mpm
