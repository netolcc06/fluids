#pragma once

namespace fluids::sph {

// Live-tunable WCSPH parameters. Plain floats only (no derived/cached
// state) so this binds as a trivial Embind value_object and can be
// reconstructed/reassigned wholesale from JS every frame if desired.
struct SPHParams {
    // Gravitational acceleration applied along +y each step, m/s^2. The
    // domain's y-axis points up (see Scenario), so the default -9.81 pulls
    // particles toward -y, i.e. down. Flip the sign for "upward gravity".
    float gravity = -9.81f;

    // Rest ("target") density, kg/m^2 (this is a 2D solver — density is
    // areal, not volumetric — but 1000 is used as a familiar water-like
    // default). Drives the Tait pressure response: particles denser than
    // this are pushed apart, particles sparser are (mildly) pulled together.
    float restDensity = 1000.0f;

    // Tait equation-of-state stiffness constant k. Higher = stiffer/less
    // compressible water, but requires a smaller stable dt.
    float gasStiffness = 200.0f;

    // Laplacian viscosity coefficient. 0 = inviscid (jittery, splashy);
    // larger values smooth out relative velocity between neighbors.
    float viscosity = 0.08f;

    // Smoothing radius h, meters: the kernel support radius and the
    // UniformGrid cell size used for neighbor search. Larger = smoother but
    // more expensive (more neighbors per particle) and more damped.
    float smoothingRadius = 0.045f;

    // Per-particle mass, kg. This MUST be calibrated against restDensity
    // and the scenario's particle spacing, or density estimation never
    // reaches restDensity, pressure is permanently clamped to 0 (see
    // SPHSolver::computeDensityPressure), and particles fall through each
    // other with no pressure repulsion at all. Rule of thumb for a
    // rest-packed 2D grid at spacing `s`: particleMass ~= restDensity * s^2
    // (mass of one grid cell's worth of area at rest density). The default
    // here (1.2) is calibrated for restDensity=1000 and the presets' default
    // spacing (0.035): 1000 * 0.035^2 = 1.225, rounded down slightly.
    float particleMass = 1.2f;

    // Fraction of velocity retained (after sign flip) when a particle
    // bounces off a domain wall, in [0, 1]. 0 = fully inelastic (stops dead
    // at the wall), 1 = perfectly elastic bounce.
    float boundaryDamping = 0.5f;
};

} // namespace fluids::sph
