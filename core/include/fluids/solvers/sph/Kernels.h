#pragma once

#include <algorithm>
#include <cmath>

#include "fluids/common/Vec2.h"

// 2D SPH smoothing kernels (Müller et al. 2003, adapted to 2D normalization
// constants). Header-only: these are tiny, called O(n * neighbors) times per
// step, and are meant to be inlined into the solver's hot loops rather than
// living behind a call boundary.
namespace fluids::sph {

constexpr float kPi = 3.14159265358979323846f;

// Minimum *effective* interaction distance used inside the density (poly6)
// and pressure-gradient (spiky) kernel formulas below, as a fraction of the
// smoothing radius h. Two particles closer than this are, for kernel-value
// purposes only, treated as if they were exactly this far apart.
//
// Why this exists (root-caused via instrumented long-run diagnostics on the
// Dam Break preset, see the settling regression test in
// tests/core/test_solver.cpp for the reproduction): a resting bed of
// particles gradually accumulates a handful of near-perfectly-coincident
// neighbor pairs (from boundary clamping + normal packing pressure — this
// grows monotonically over a run and never reverses without this fix). Both
// poly6 and the spiky gradient's magnitude are mathematically BOUNDED as
// r -> 0 (not a literal 1/r divergence — the r in spikyGradient's
// denominator exactly cancels the r embedded in rVec's own magnitude, so
// the returned vector's length tops out at a fixed, finite value, and
// poly6(r,h) is a smooth polynomial in r^2 with a finite max at r=0). The
// real problem is that poly6 is nearly FLAT for r << h (it only falls off
// sharply as r approaches h), so a single near-coincident pair contributes
// almost its absolute maximum density value — many times more than a pair
// at the normal rest spacing would. That inflated density, run through the
// Tait EOS's exponent-7 stiffness response, produces a wildly
// disproportionate pressure, which combined with the spiky gradient's
// (bounded but, at small r, near-its-own-maximum) magnitude yields a real,
// finite, but large (measured: several hundred to ~2000 m/s^2, vs ~10 m/s^2
// from gravity) per-step acceleration kick — enough to launch a resting
// particle to double-digit m/s in a single 1/120s substep. Because nothing
// currently pulls exactly-coincident pairs back apart (spikyGradient's old
// r<1e-6 guard returned a hard zero there), this is a one-way ratchet: the
// population of near-coincident pairs only grows over time, so the bad
// kernel values keep occurring and the "rain" of launched particles never
// stops.
//
// Fix: floor the effective r fed into both kernels' formulas (but not the
// r>h support check, nor the direction rVec/r, which stays keyed to the
// true r so direction is unaffected once r is safely away from 0). 0.5*h
// is chosen empirically (see diagnostics referenced above) as the smallest
// fraction that meaningfully bounds poly6's near-r=0 plateau: poly6 falls
// to ~42% of its unclamped r=0 max at r=0.5h (vs ~97% still remaining at a
// naive-looking r=0.1h, which barely helps since poly6 is that flat near
// the origin), while 0.5h is still comfortably below the built-in presets'
// spacing/h ratio (~0.78, i.e. normal rest-packed neighbors sit at r ~ h,
// well outside the floor) so ordinary resting/compressed configurations are
// completely unaffected — only the pathological near-total-overlap regime
// is reshaped.
constexpr float kMinEffectiveRFraction = 0.5f;

// Poly6 kernel value, used for density estimation. Smooth (C2), maximal at
// r=0, monotonically decreasing to exactly 0 at r=h, and 0 beyond (r<0 is
// treated as invalid input and also returns 0 defensively). The effective r
// used in the formula is floored at kMinEffectiveRFraction * h — see the
// comment above kMinEffectiveRFraction for why.
inline float poly6(float r, float h) {
    if (r < 0.0f || r > h) return 0.0f;
    const float rEff = std::max(r, kMinEffectiveRFraction * h);
    const float h2 = h * h;
    const float r2 = rEff * rEff;
    const float diff = h2 - r2;
    const float coeff = 4.0f / (kPi * std::pow(h, 8.0f));
    return coeff * diff * diff * diff;
}

// Gradient of the Spiky kernel, used for the pressure force. `rVec` is the
// displacement between the two particles (e.g. pos_i - pos_j) and `r` is
// its length; the two are passed separately so callers that already
// computed r (for a neighbor-radius check) don't pay for sqrt twice.
//
// Direction is rVec / r using the TRUE r (not floored) — dividing by the
// real r is exactly what turns rVec into a unit vector, and that division
// is well-conditioned for any r > 0 no matter how small, since rVec's own
// magnitude shrinks alongside r (their ratio stays a proper unit vector).
// Only r == 0 exactly is a genuine 0/0 singularity (direction is truly
// undefined when two particles occupy the exact same float32 position);
// that case alone returns the zero vector, same as the old code's intent,
// but WITHOUT the old code's much wider (and, per the diagnosis above,
// actively harmful) dead zone below r=1e-6.
//
// The force MAGNITUDE, by contrast, is computed from the effective r
// floored at kMinEffectiveRFraction * h (see the comment above that
// constant) — flooring only the magnitude term (not the direction
// division) is what keeps a very-close-but-nonzero pair correctly and
// boundedly repulsive instead of either an oversized spike or a hard zero.
//
// Returns the zero vector when r > h (outside the kernel support) or r<=0
// (coincident / invalid input).
inline Vec2 spikyGradient(const Vec2& rVec, float r, float h) {
    if (r <= 0.0f || r > h) return Vec2{0.0f, 0.0f};
    const float rEff = std::max(r, kMinEffectiveRFraction * h);
    const float coeff = -30.0f / (kPi * std::pow(h, 5.0f));
    const float term = h - rEff;
    // (coeff * term^2) is dW/dr evaluated at the floored effective
    // distance — this is the part that's now bounded well below its old
    // r->0 ceiling.
    const float dWdr = coeff * term * term;
    return rVec * (dWdr / r);
}

// Laplacian of the (Müller) viscosity kernel, used for the viscosity force.
// Positive and maximal at r=0, decreasing linearly to 0 at r=h.
inline float viscosityLaplacian(float r, float h) {
    if (r < 0.0f || r > h) return 0.0f;
    const float coeff = 40.0f / (kPi * std::pow(h, 5.0f));
    return coeff * (h - r);
}

} // namespace fluids::sph
