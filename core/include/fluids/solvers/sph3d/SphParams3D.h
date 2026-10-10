#pragma once

namespace fluids::sph3d {

// Live-tunable 3D WCSPH parameters, the 3D sibling of fluids::sph::SPHParams
// (see SPHParams.h). Same fields, same meanings, adapted to 3D where the
// physics is genuinely dimension-dependent — plain floats only (no
// derived/cached state) so this binds as a trivial Embind value_object.
//
// Defaults here are NOT the 2D defaults with restDensity/spacing/mass
// swapped in blind — they were determined empirically (native diagnostic:
// seed a rest-packed 3D grid at candidate spacing/h, measure the resulting
// Poly6 density estimate, and run short dam-break simulations at candidate
// gasStiffness/dt to check for stability) because 3D has fundamentally more
// neighbors within the same smoothing radius (a sphere's volume grows with
// h^3 vs. a circle's h^2), which changes both the density estimate's scale
// and the pressure/viscosity force sums per particle. See
// docs/sph3d-sim-api.md for the full derivation and the diagnostic results.
struct SphParams3D {
    // Gravitational acceleration applied along +y each step, m/s^2. Same
    // convention as 2D: the domain's y-axis points up, so the default -9.81
    // pulls particles toward -y, i.e. down.
    float gravity = -9.81f;

    // Rest ("target") density, kg/m^3 (this IS volumetric in 3D, unlike the
    // 2D solver's areal kg/m^2 — see the particleMass note below). 1000 is
    // used as the familiar water-like default, same numeric value as 2D
    // even though the units are now genuinely different (areal vs.
    // volumetric) — it's restDensity's role (the target the Tait EOS pushes
    // density estimates toward) that's dimension-independent, not its
    // numeric value being "the same physical quantity" as the 2D one.
    float restDensity = 1000.0f;

    // Tait equation-of-state stiffness constant k. Higher = stiffer/less
    // compressible water, but requires a smaller stable dt. This acts only
    // on the dimensionless ratio density/restDensity (see SPHSolver3D's
    // Tait EOS application), so — confirmed empirically, see
    // docs/sph3d-sim-api.md — it does not need the same kind of rescaling
    // particleMass/smoothingRadius do; 200 (unchanged from 2D) was
    // re-verified stable at the dt below.
    float gasStiffness = 200.0f;

    // Laplacian viscosity coefficient. 0 = inviscid (jittery, splashy);
    // larger values smooth out relative velocity between neighbors. Kept
    // equal to the 2D default (0.08) — empirically it produced comparable
    // damping behavior in 3D once particleMass/smoothingRadius were
    // recalibrated; the per-neighbor viscosity force scales with the same
    // particleMass/density normalization as pressure, so it tracks the
    // other recalibrated constants rather than needing its own.
    float viscosity = 0.08f;

    // Smoothing radius h, meters: the kernel support radius and the
    // UniformGrid3D cell size used for neighbor search. Larger in absolute
    // terms than the 2D default (0.045) at a similar spacing/h ratio (0.06 /
    // 0.09 = 0.667, vs. 2D's 0.035 / 0.045 = 0.778 — slightly coarser so
    // each particle still sees a full neighborhood despite the 3D particle
    // count growing with the cube, not the square, of resolution). See the
    // "accept coarse grids for performance" project-wide decision and
    // docs/sph3d-sim-api.md for the empirical tuning that landed here.
    float smoothingRadius = 0.09f;

    // Per-particle mass, kg. MUST be calibrated against restDensity and the
    // scenario's particle spacing, same failure mode as 2D if it isn't (see
    // SphSolver3D::computeDensityPressure): density estimation never reaches
    // restDensity, pressure is permanently clamped to 0, particles fall
    // through each other with no repulsion. Baseline rule of thumb for a
    // rest-packed 3D grid at spacing `s` — note the CUBE, not the square the
    // 2D solver uses, because a rest-packed grid cell is now a volume
    // element, not an area element:
    //   particleMass ~= restDensity * s^3
    // For restDensity=1000 and the presets' default spacing (0.06), that
    // naive formula gives 1000 * 0.06^3 = 0.216 — but verified empirically
    // (native diagnostic: tests/core/test_solver3d.cpp's calibration test
    // sums the 3D Poly6 kernel over an infinite rest-packed lattice at this
    // spacing/smoothingRadius) that only measures a density of ~681, about
    // 32% short of restDensity. Unlike 2D (where the naive formula alone
    // landed close enough), 3D's h/spacing=1.5 ratio here only captures the
    // face+edge neighbor shells (19 neighbors total within h, not a full
    // sphere's worth) before the kernel support cuts off, so the naive
    // formula under-integrates. The fix applied here is a measured
    // correction factor (~1.47x the naive value) on top of the s^3 rule,
    // not a change to smoothingRadius/spacing themselves (those were
    // already chosen for the "accept coarse grids for performance"
    // trade-off): particleMass = 1000 * 0.06^3 * 1.468 = 0.317, which
    // measures within 1% of restDensity on the same lattice (see
    // docs/sph3d-sim-api.md for the full derivation and the numbers). If
    // you change spacing/smoothingRadius, recompute this the same way
    // (rerun the lattice-sum calibration, don't just reapply 1.468x blindly
    // — that factor is itself a function of the h/spacing ratio).
    float particleMass = 0.317f;

    // Fraction of velocity retained (after sign flip) when a particle
    // bounces off a domain wall, in [0, 1]. Same meaning and default as 2D.
    float boundaryDamping = 0.5f;
};

} // namespace fluids::sph3d
