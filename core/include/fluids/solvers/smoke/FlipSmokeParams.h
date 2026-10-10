#pragma once

namespace fluids::smoke {

// Live-tunable Stable Fluids (Jos Stam) smoke parameters. Deliberately a
// different shape from flip::FlipParams: there is no `gravity` field at all
// (buoyancy — driven by temperature — replaces gravity as the force that
// moves this material, see `buoyancy` below) and no `flipRatio` (there are
// no particles, so there is no PIC/FLIP blend to speak of; velocity lives
// only on the MAC grid and is self-advected every step).
//
// Temperature is a dimensionless *relative* scale, not physical degrees:
// `ambientTemperature = 0` and `sourceTemperature = 1` is the default
// convention used throughout this solver (see FlipSmokeSolver.cpp). Nothing
// stops a caller from using a different scale (e.g. real Kelvin deltas) —
// only relative differences from `ambientTemperature` ever matter, since
// they only ever appear as `(T - ambientTemperature)` in the buoyancy term
// and as a relaxation target in dissipation.
struct FlipSmokeParams {
    // Buoyancy coefficient, (m/s^2) per unit temperature above ambient.
    // Applied to vertical (v) face velocities each step as
    // `v += dt * buoyancy * (T_face - ambientTemperature)`, where T_face is
    // the average of the two cell-centered temperatures adjacent to that
    // face (see FlipSmokeSolver::applyBuoyancy). Positive values make
    // hotter-than-ambient smoke rise (+y is up, same convention as
    // flip::FlipParams::gravity) — this is the sole driving force for
    // smoke; there is no separate gravity parameter because a uniform
    // downward pull on every cell alike would just be a constant offset
    // negative buoyancy contributes zero net motion without a temperature
    // difference. Default `4.0` gives a visible, unhurried rise for the
    // default cellSize/domain — see docs/flip-smoke-sim-api.md for the
    // empirical substep/parameter interaction notes.
    float buoyancy = 4.0f;

    // Relative temperature of undisturbed air, far from the source. Acts as
    // the zero point buoyancy measures against and the value temperature
    // dissipates toward (see temperatureDissipation). Default `0.0` (see
    // the dimensionless-scale note above).
    float ambientTemperature = 0.0f;

    // Relative temperature injected into cells inside the scenario's source
    // region every step (see SmokeScenario and FlipSmokeSolver::emitSource).
    // Default `1.0`, i.e. one unit hotter than ambient — combined with the
    // default `buoyancy`, this is what makes the plume rise.
    float sourceTemperature = 1.0f;

    // Fractional density loss per second: each step,
    // `density *= (1 - densityDissipation * dt)`, clamped so density never
    // goes negative. Without this, continuously-emitted smoke would
    // accumulate in the domain forever (density is not otherwise removed —
    // there's no outflow boundary). Default `0.1` (10%/s) — slow enough
    // that the plume shape stays visible for several seconds, fast enough
    // that the domain doesn't uniformly saturate to the density cap over a
    // long-running scene.
    float densityDissipation = 0.1f;

    // Fractional relaxation of temperature toward `ambientTemperature` per
    // second: each step, `T += (ambientTemperature - T) *
    // min(1, temperatureDissipation * dt)`. Without this, buoyancy would
    // keep accelerating already-hot cells indefinitely as they rise (no
    // physical heat loss to the surrounding "air" the way a real plume
    // cools as it mixes). Default `0.5` (temperature halves its distance
    // from ambient roughly every ~1.4s) — noticeably faster than
    // densityDissipation, matching the real-world behavior that a plume's
    // visible smoke (density) lingers longer than its buoyant "hotness"
    // (temperature), so it eventually drifts/settles rather than rising
    // forever.
    float temperatureDissipation = 0.5f;

    // MAC grid cell size, meters. Same role and reset()-only-applies
    // contract as flip::FlipParams::cellSize: grid resolution (nx, ny) is
    // derived from SmokeScenario::domainWidth/domainHeight and this value,
    // and is only (re)allocated on FlipSmokeSolver::reset() /
    // FlipSmokeSimulation::reset() — changing it without calling reset()
    // afterward has no effect until the next reset(). Default `0.02` gives
    // roughly a 60x120 grid for the default ~1.2m x 2.4m makePlumeScenario()
    // domain (see docs/flip-smoke-sim-api.md for the performance-cliff
    // discussion, same shape as FLIP water's).
    float cellSize = 0.02f;
};

} // namespace fluids::smoke
