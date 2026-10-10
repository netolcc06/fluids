#pragma once

#include <cstddef>
#include <vector>

#include "fluids/common/Mat2.h"
#include "fluids/common/Vec2.h"
#include "fluids/scenario/Scenario.h"
#include "fluids/solvers/mpm/MpmGrid.h"
#include "fluids/solvers/mpm/MpmParams.h"

namespace fluids::mpm {

// MLS-MPM (Moving Least Squares Material Point Method) water solver, using
// APIC (Affine Particle-In-Cell) transfer and a weakly-compressible fluid
// constitutive model — the standard "88-line MLS-MPM" fluid recipe (Hu et
// al. / Taichi), adapted to standalone 2D C++.
//
// Per-particle state: position, velocity, the APIC affine velocity matrix C
// (2x2), and a scalar J (determinant of the deformation gradient). Only J is
// tracked (not a full deformation gradient matrix) because a fluid has no
// shear elasticity — isotropic volume change is all that matters for the
// pressure response.
//
// Per-step pipeline (see .cpp for the derivation behind each stage):
//   1. Clear the grid (mass + momentum).
//   2. P2G: quadratic-B-spline-weighted scatter of mass, momentum, and the
//      EOS-pressure-derived affine stress contribution onto the 3x3 node
//      neighborhood around each particle.
//   3. Grid update: momentum -> velocity, apply gravity, margin-based
//      boundary velocity clamp.
//   4. G2P: gather grid velocities back with the same weights to get the
//      particle's new velocity and APIC affine matrix C; update J from
//      trace(C); advect position; particle-level boundary clamp.
//
// Same statelessness contract as SPHSolver/FlipSolver: step() takes
// MpmParams by const reference and reads it fresh every call, EXCEPT
// cellSize, which only takes effect via reset() (grid (re)allocation only
// ever happens there, matching FlipParams::cellSize's contract).
//
// step() internally subdivides the caller's dt into one or more CFL-safe
// sub-steps (see the .cpp for the exact bound) — a single coherent mass
// hitting the floor at a few m/s (e.g. the Block Drop preset) can otherwise
// produce a velocity gradient across the impact interface large enough to
// swing a particle's J by a large fraction in one step, which the
// stiff EOS then amplifies into a runaway pressure spike within a handful of
// steps. This keeps the public contract "step(dt) advances by dt seconds"
// unchanged — see docs/mpm-sim-api.md for the caller-facing writeup.
class MpmSolver {
public:
    void reset(const Scenario& scenario, const MpmParams& params);
    void step(const MpmParams& params, float dt);

    std::size_t particleCount() const { return positions_.size(); }
    const std::vector<Vec2>& positions() const { return positions_; }
    const std::vector<Vec2>& velocities() const { return velocities_; }

    // Read-only access for native tests/debugging — not part of the JS
    // contract (only MpmSimulation is bound to Embind).
    const MpmGrid& grid() const { return grid_; }
    const std::vector<float>& J() const { return J_; }
    float particleMass() const { return particleMass_; }

private:
    Scenario scenario_;
    std::vector<Vec2> positions_;
    std::vector<Vec2> velocities_;
    std::vector<Mat2> C_;
    std::vector<float> J_;

    // Per-particle mass and rest volume, calibrated once at reset() from
    // params.restDensity and scenario.spacing (see MpmSolver::reset — same
    // particleMass ~= restDensity * spacing^2 rule as SPH). All particles
    // share the same value since they're all seeded on a single regular
    // grid at one spacing.
    float particleMass_ = 0.0f;
    float restVolume_ = 0.0f;

    MpmGrid grid_;

    void seedRegion(float x, float y, float width, float height, float vx, float vy);

    void particleToGrid(const MpmParams& params, float dt);
    void updateGrid(const MpmParams& params, float dt);
    void gridToParticles(float dt);
    void clampToDomain();

    // Runs one full clear/P2G/grid-update/G2P/clamp pass at the given
    // (already-safe) dt. step() calls this 1+ times per caller-requested dt,
    // subdividing internally for stability — see step()'s comment.
    void substep(const MpmParams& params, float dt);

    // Largest per-particle speed currently in velocities_ (0 if there are no
    // particles). Used by step() to pick a CFL-safe internal sub-dt.
    float maxParticleSpeed() const;
};

} // namespace fluids::mpm
