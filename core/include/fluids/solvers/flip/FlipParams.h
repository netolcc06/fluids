#pragma once

namespace fluids::flip {

// Live-tunable PIC/FLIP parameters. Deliberately a different shape from
// sph::SPHParams — FLIP has no particle-mass/density-calibration machinery
// to keep in sync with scenario spacing (incompressibility comes from the
// grid pressure projection, not an equation of state), so there's no
// restDensity/particleMass pair here at all.
struct FlipParams {
    // Gravitational acceleration applied to grid velocities along +y each
    // step, m/s^2. Same convention as sph::SPHParams::gravity: domain +y is
    // up, so the default -9.81 pulls fluid down.
    float gravity = -9.81f;

    // PIC/FLIP blend factor, in [0, 1] (clamped defensively in
    // FlipSolver::step). 0 = pure PIC: each step, a particle's velocity is
    // fully replaced by the grid velocity interpolated at its position —
    // very stable/damped (numerical viscosity from repeated grid transfer),
    // "thick"-looking water. 1 = pure FLIP: a particle keeps its own
    // velocity and only adds the grid's *change* in velocity since the
    // last P2G transfer — energetic/splashy, but more prone to visible
    // noise since nothing damps small errors introduced by the (Jacobi,
    // finite-iteration) pressure solve. 0.95 — mostly FLIP with a sliver of
    // PIC damping to keep that noise in check — is the standard
    // recommendation and the default here.
    float flipRatio = 0.95f;

    // MAC grid cell size, meters. Grid resolution (nx, ny) is derived from
    // Scenario::domainWidth/domainHeight and this value, and is only
    // (re)allocated on reset() — changing cellSize without calling reset()
    // again has no effect until the next reset(). Smaller = higher
    // resolution (sharper free surface, better pressure solve) but cost
    // scales roughly with cell count (quadratic in 1/cellSize for a fixed
    // domain), and the fixed-iteration Jacobi pressure solve needs
    // proportionally more iterations to fully converge on a bigger grid —
    // so very small values (well under ~0.01 for a ~2m-wide domain, i.e.
    // 200+ cells/side) can blow the real-time budget in single-threaded
    // WASM. Default 0.025 gives an ~80x60 grid for the built-in ~2x1.5m
    // scenarios.
    float cellSize = 0.025f;
};

} // namespace fluids::flip
