#pragma once

#include "fluids/common/Vec2.h"

namespace fluids {

// Minimal 2x2 matrix, row-major (m00 m01; m10 m11). Added for Phase 3
// (MLS-MPM): the APIC affine velocity field `C`, the per-particle stress
// tensor, and the P2G affine-momentum term all need a small dense 2x2 type.
// Not used by SPH or FLIP — kept in common/ purely because it's a generic
// math primitive like Vec2, not because anything outside mpm/ needs it yet.
struct Mat2 {
    float m00 = 0.0f, m01 = 0.0f;
    float m10 = 0.0f, m11 = 0.0f;

    Mat2() = default;
    Mat2(float a00, float a01, float a10, float a11) : m00(a00), m01(a01), m10(a10), m11(a11) {}

    static Mat2 identity() { return Mat2(1.0f, 0.0f, 0.0f, 1.0f); }
    static Mat2 zero() { return Mat2(0.0f, 0.0f, 0.0f, 0.0f); }

    Mat2 operator+(const Mat2& rhs) const {
        return Mat2(m00 + rhs.m00, m01 + rhs.m01, m10 + rhs.m10, m11 + rhs.m11);
    }
    Mat2 operator-(const Mat2& rhs) const {
        return Mat2(m00 - rhs.m00, m01 - rhs.m01, m10 - rhs.m10, m11 - rhs.m11);
    }
    Mat2 operator*(float s) const { return Mat2(m00 * s, m01 * s, m10 * s, m11 * s); }

    // Matrix-vector product (this * v).
    Vec2 operator*(const Vec2& v) const { return Vec2(m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y); }

    Mat2 transpose() const { return Mat2(m00, m10, m01, m11); }
    float trace() const { return m00 + m11; }
};

inline Mat2 operator*(float s, const Mat2& m) { return m * s; }

// Outer product a * b^T, i.e. result(i,j) = a[i] * b[j]. `a` first matches
// the MLS-MPM G2P formula's outer(velocity, dpos) convention.
inline Mat2 outer(const Vec2& a, const Vec2& b) { return Mat2(a.x * b.x, a.x * b.y, a.y * b.x, a.y * b.y); }

} // namespace fluids
