#pragma once

namespace fluids::sph_smoke {

// Live-tunable parameters for the SPH-flavored smoke solver (Phase 4c).
// Field shape deliberately mirrors fluids::mpm_smoke::MpmSmokeParams /
// fluids::smoke::FlipSmokeParams wherever the underlying physics is the same
// concept (buoyancy, ambient/source temperature, density/temperature
// dissipation, maxParticles) — all three smoke variants should feel like the
// same knobs applied to a different transfer/discretization scheme, not an
// unrelated parameter set. `smoothingRadius` is this solver's genuinely new
// field, playing the same "this solver's spatial resolution knob" role that
// `cellSize` plays for the two grid/APIC-based solvers.
//
// No `gravity` field, same reasoning as every other smoke variant in this
// project: buoyancy (driven by each particle's own temperature relative to
// ambient) is smoke's sole driving force.
//
// No `gasStiffness`/`restDensity` the way sph::SPHParams (water) has them —
// this solver deliberately drops the pressure/EOS force entirely, same
// design choice mpm_smoke::MpmSmokeParams made for its stress term, for the
// same underlying reason: smoke's particle population is fundamentally
// sparse/continuously-thinning (emitted, spreading, dissipating) rather than
// packed near a rest density the way water's Tait EOS assumes, and a
// water-calibrated pressure response was found (see
// docs/sph-smoke-sim-api.md's "Pressure/EOS decision" section) to add
// instability risk with no corresponding visual benefit — coherent,
// smoke-like spreading comes entirely from buoyancy plus the kernel-weighted
// viscosity term below. Per-particle mass for the density/viscosity kernel
// sums is a fixed internal constant (see SphSmokeSolver.cpp's kParticleMass),
// not a tunable, because (like MpmSmokeParams' particle mass) it factors out
// of every ratio this solver actually computes — see SphSmokeSolver.cpp for
// the derivation.
struct SphSmokeParams {
    // Buoyancy coefficient, (m/s^2) per unit temperature above ambient.
    // Applied directly to each particle's vertical acceleration every step:
    // `accel.y += buoyancy * (particle.temperature - ambientTemperature)`.
    // Same formula shape and default as FlipSmokeParams::buoyancy /
    // MpmSmokeParams::buoyancy, evaluated per-particle. This is smoke's
    // entire driving force — there is no separate gravity parameter.
    float buoyancy = 4.0f;

    // Relative temperature of undisturbed air, far from the source. Zero
    // point buoyancy measures against, and the value each particle's
    // temperature relaxes toward (see temperatureDissipation). Default
    // `0.0`, same convention as every other smoke variant.
    float ambientTemperature = 0.0f;

    // Relative temperature newly-emitted particles are seeded near (jittered
    // slightly below this, see SphSmokeSolver::emit) every step within the
    // scenario's source region. Default `1.0`, matching the other two smoke
    // variants — combined with the default buoyancy, this is what makes
    // emitted particles rise.
    float sourceTemperature = 1.0f;

    // Fractional density loss per second, applied per particle:
    // `density *= (1 - densityDissipation * dt)`, clamped so density never
    // goes negative. Same formula and default as the other smoke variants.
    // Also indirectly controls how long a particle survives before falling
    // below the culling threshold and being removed — see
    // docs/sph-smoke-sim-api.md's "Culling" section.
    float densityDissipation = 0.1f;

    // Fractional relaxation of temperature toward `ambientTemperature` per
    // second, applied per particle: `T += (ambientTemperature - T) *
    // min(1, temperatureDissipation * dt)`. Same formula and default as the
    // other two smoke variants — deliberately faster than
    // densityDissipation so a particle's buoyant "hotness" fades before its
    // visible density trail does.
    float temperatureDissipation = 0.5f;

    // SPH kernel support radius h, meters — this solver's analog of
    // `cellSize` in the grid/APIC-based solvers: it sets both the
    // neighbor-search UniformGrid cell size and the poly6/viscosityLaplacian
    // kernel support. Unlike sph::SPHParams::smoothingRadius (water, 0.045m,
    // tuned for a densely rest-packed fluid), this default is noticeably
    // larger (see docs/sph-smoke-sim-api.md for the empirical reasoning) —
    // smoke's particle population is much sparser than water's, so a larger
    // radius is needed for a typical particle to actually have a handful of
    // neighbors to smooth against. Read fresh every step() (grid rebuilt
    // every step regardless, same as water SPH) — unlike cellSize elsewhere
    // in this project, this does NOT require a reset() to take effect.
    float smoothingRadius = 0.09f;

    // Kernel-based (Laplacian) viscosity coefficient — the coherence/
    // diffusion knob that gives this solver its "genuine SPH character"
    // (local kernel-weighted velocity averaging between neighbors), reusing
    // fluids::sph::Kernels.h's viscosityLaplacian unchanged from water SPH.
    // Deliberately a much larger default than sph::SPHParams::viscosity
    // (0.08): with no pressure force to keep particles apart or lend the
    // simulation any of its own coherence, viscosity here is doing
    // significantly more of the visual work of making a rising particle
    // cloud look like a single coherent puff of smoke rather than a
    // scattered dust of independent points (see
    // docs/sph-smoke-sim-api.md for the empirical tuning notes). `0` makes
    // this a completely inertialess ballistic particle spray (buoyancy +
    // jitter only, no inter-particle coupling at all).
    float viscosity = 6.0f;

    // Fixed capacity of the particle buffer, (re)allocated only at reset()
    // (same reset()-only-applies contract as maxParticles in
    // MpmSmokeParams). Continuous emission is otherwise unbounded, so this
    // both bounds worst-case per-step cost (an O(activeCount * neighbors)
    // grid rebuild + two neighbor-query passes every step, same shape as
    // water SPH's per-step cost) and backstops the buffer-pointer-stability
    // contract — see SphSmokeSolver's class comment and
    // docs/sph-smoke-sim-api.md. Default `4000`, same default as
    // MpmSmokeParams, comfortably above this solver's empirically measured
    // steady-state population at default settings (see
    // docs/sph-smoke-sim-api.md).
    int maxParticles = 4000;
};

} // namespace fluids::sph_smoke
