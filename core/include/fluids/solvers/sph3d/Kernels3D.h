#pragma once

#include <algorithm>
#include <cmath>

#include "fluids/common/Vec3.h"

// 3D SPH smoothing kernels (Muller et al. 2003 normalization constants,
// adapted to 3D). Header-only, same rationale as the 2D Kernels.h: tiny,
// called O(n * neighbors) times per step, meant to be inlined into the
// solver's hot loops.
//
// These are NOT copy-pasted from the 2D Kernels.h with the constants left
// alone — the normalization constants are genuinely dimension-specific
// (they come from integrating the kernel shape over a disk vs. a ball), so
// this is an independent derivation using the standard 3D constants:
//   - poly6:  315 / (64 * pi * h^9)
//   - spiky gradient magnitude coefficient: 45 / (pi * h^6)
//   - viscosity laplacian: 45 / (pi * h^6)
// (2D uses 4/(pi h^8), 30/(pi h^5), 40/(pi h^5) respectively — different
// exponents on h and different leading constants, as expected.)
namespace fluids::sph3d {

constexpr float kPi = 3.14159265358979323846f;

// Same fix as the 2D solver's kMinEffectiveRFraction (see
// core/include/fluids/solvers/sph/Kernels.h for the full derivation/history
// of the bug this prevents — a resting bed of particles accumulating
// near-coincident neighbor pairs whose poly6-driven density estimate spikes
// disproportionately near r=0, which the Tait EOS's 7th-power response then
// amplifies into a large, one-way-ratcheting force kick that never lets the
// bed go calm). Baked into this 3D implementation from day one rather than
// waiting to rediscover the same bug: both poly6() and spikyGradient() floor
// the *effective* interaction distance used inside their magnitude formulas
// at 0.5 * smoothingRadius (direction is unaffected — spikyGradient still
// divides by the true, unfloored r to build the unit vector). The same 0.5
// choice is reused here since the underlying shape of the problem (a
// near-flat kernel plateau close to r=0, feeding an exponent-7 stiffness
// response) is dimension-independent; only the normalization constants in
// front of the kernels differ between 2D and 3D.
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
    const float coeff = 315.0f / (64.0f * kPi * std::pow(h, 9.0f));
    return coeff * diff * diff * diff;
}

// Gradient of the Spiky kernel, used for the pressure force. `rVec` is the
// displacement between the two particles (e.g. pos_i - pos_j) and `r` is its
// length; the two are passed separately so callers that already computed r
// (for a neighbor-radius check) don't pay for sqrt twice.
//
// Direction is rVec / r using the TRUE r (not floored) — same reasoning as
// the 2D spikyGradient: this division is well-conditioned for any r > 0 no
// matter how small, since rVec's own magnitude shrinks alongside r. Only
// r == 0 exactly is a genuine 0/0 singularity (direction truly undefined);
// that case alone returns the zero vector.
//
// The force MAGNITUDE is computed from the effective r floored at
// kMinEffectiveRFraction * h (see kMinEffectiveRFraction above) — flooring
// only the magnitude term (not the direction division) keeps a
// very-close-but-nonzero pair correctly and boundedly repulsive instead of
// either an oversized spike or a hard zero.
//
// Returns the zero vector when r > h (outside the kernel support) or r<=0
// (coincident / invalid input).
inline Vec3 spikyGradient(const Vec3& rVec, float r, float h) {
    if (r <= 0.0f || r > h) return Vec3{0.0f, 0.0f, 0.0f};
    const float rEff = std::max(r, kMinEffectiveRFraction * h);
    const float coeff = -45.0f / (kPi * std::pow(h, 6.0f));
    const float term = h - rEff;
    // (coeff * term^2) is dW/dr evaluated at the floored effective distance
    // — this is the part that's now bounded well below its old r->0 ceiling.
    const float dWdr = coeff * term * term;
    return rVec * (dWdr / r);
}

// Laplacian of the (Muller) viscosity kernel, used for the viscosity force.
// Positive and maximal at r=0, decreasing linearly to 0 at r=h. No
// near-coincidence floor is needed here: unlike poly6/spiky, this term is
// already linear in r (not raised through an exponent-7 EOS), so it never
// produced the runaway force spike the other two did.
inline float viscosityLaplacian(float r, float h) {
    if (r < 0.0f || r > h) return 0.0f;
    const float coeff = 45.0f / (kPi * std::pow(h, 6.0f));
    return coeff * (h - r);
}

} // namespace fluids::sph3d
