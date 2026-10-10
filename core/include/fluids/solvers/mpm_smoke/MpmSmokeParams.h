#pragma once

namespace fluids::mpm_smoke {

// Live-tunable parameters for the MPM-flavored (particle/APIC) smoke solver.
// Field shape deliberately mirrors fluids::smoke::FlipSmokeParams wherever
// the underlying physics is the same concept (buoyancy, ambient/source
// temperature, density/temperature dissipation, cellSize) — the two solvers
// should feel like the same knobs applied to a different transfer scheme,
// not an unrelated parameter set. Genuinely new fields (maxParticles,
// damping) are MPM-particle-specific and have no FlipSmokeParams analog.
//
// No `gravity` field, same reasoning as FlipSmokeParams: buoyancy (driven by
// each particle's own temperature relative to ambient) is smoke's sole
// driving force in this project, for both the grid (Phase 4a) and MPM
// (Phase 4b) flavors.
//
// No `restDensity`/`stiffness`/`viscosity` the way mpm::MpmParams (water)
// has them, because this solver drops the EOS/pressure stress term
// entirely — see fluids::mpm_smoke::MpmSmokeSolver's class comment for why
// (smoke here is a compressible buoyant gas carried by particles, not an
// incompressible fluid resisting compression). Per-particle mass for the
// APIC transfer is therefore a fixed internal constant, not a tunable
// (see MpmSmokeSolver.cpp's kParticleMass) — with no stress force, its
// absolute value is arbitrary and cancels out of the P2G/G2P
// momentum-weighted average, so there's nothing meaningful to expose.
struct MpmSmokeParams {
    // Buoyancy coefficient, (m/s^2) per unit temperature above ambient.
    // Applied directly to each particle's vertical velocity every step:
    // `v.y += dt * buoyancy * (particle.temperature - ambientTemperature)`.
    // Same formula shape and default as FlipSmokeParams::buoyancy, just
    // evaluated per-particle (using that particle's own temperature) instead
    // of per-grid-face. This is smoke's entire driving force — there is no
    // separate gravity parameter (see class comment).
    float buoyancy = 4.0f;

    // Relative temperature of undisturbed air, far from the source. Zero
    // point buoyancy measures against, and the value each particle's
    // temperature relaxes toward (see temperatureDissipation). Default
    // `0.0`, same convention and default as FlipSmokeParams.
    float ambientTemperature = 0.0f;

    // Relative temperature newly-emitted particles are seeded near (jittered
    // slightly below this, see MpmSmokeSolver::emit) every step within the
    // scenario's source region. Default `1.0`, matching FlipSmokeParams —
    // combined with the default buoyancy, this is what makes emitted
    // particles rise.
    float sourceTemperature = 1.0f;

    // Fractional density loss per second, applied per particle:
    // `density *= (1 - densityDissipation * dt)`, clamped so density never
    // goes negative. Same formula and default as FlipSmokeParams. Also
    // indirectly controls how long a particle survives before falling below
    // the culling threshold and being removed (see MpmSmokeSolver's culling
    // section in docs/mpm-smoke-sim-api.md) — raising this shortens both the
    // visible fade and each particle's lifetime.
    float densityDissipation = 0.1f;

    // Fractional relaxation of temperature toward `ambientTemperature` per
    // second, applied per particle: `T += (ambientTemperature - T) *
    // min(1, temperatureDissipation * dt)`. Same formula and default as
    // FlipSmokeParams — deliberately faster than densityDissipation so a
    // particle's buoyant "hotness" fades before its visible density trail
    // does, same real-plume intuition as the grid version.
    float temperatureDissipation = 0.5f;

    // Node-centered grid spacing (dx) for the APIC P2G/G2P transfer, meters.
    // Same role and reset()-only-applies contract as mpm::MpmParams::cellSize
    // / FlipSmokeParams::cellSize: grid resolution is derived from
    // SmokeScenario::domainWidth/domainHeight and this value, and is only
    // (re)allocated on reset() — changing it without calling reset()
    // afterward has no effect until the next reset().
    float cellSize = 0.02f;

    // Optional linear velocity drag: `v *= max(0, 1 - damping * dt)`,
    // applied right after buoyancy each step. 0 (the default, used by the
    // Plume preset and all native tests) disables this entirely. MPM water's
    // optional viscosity term uses the APIC matrix C's symmetric part as a
    // cheap velocity-gradient stress estimate — that trick doesn't apply
    // here since there is no stress term at all, so a plain linear drag is
    // the simplest analog if a caller finds the particle gas looks too
    // chaotic/energetic without any damping. Empirically, the default
    // buoyancy/dissipation/emission settings did not need this to look
    // stable and reasonably coherent (see docs/mpm-smoke-sim-api.md) — it's
    // included as a knob, not because 0 looked broken.
    float damping = 0.0f;

    // Fixed capacity of the particle buffer, (re)allocated only at reset()
    // (same reset()-only-applies contract as cellSize). Continuous emission
    // is otherwise unbounded, so this both bounds worst-case per-step cost
    // in real-time single-threaded WASM and backstops the buffer-pointer-
    // stability contract (see MpmSmokeSolver's class comment and
    // docs/mpm-smoke-sim-api.md): the position/velocity/density/temperature
    // buffers are allocated at this size exactly once per reset() and never
    // resized afterward, only ever filled/read up to the current active
    // count. Default `4000` is comfortably above the steady-state particle
    // count the default emission/dissipation settings reach in practice
    // (empirically a few hundred to ~2000, see docs/mpm-smoke-sim-api.md),
    // so it acts as a safety backstop rather than a normally-hit ceiling —
    // same spirit as mpm::MpmSolver's kMaxSubsteps cap.
    int maxParticles = 4000;
};

} // namespace fluids::mpm_smoke
