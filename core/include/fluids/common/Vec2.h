#pragma once

#include <cmath>
#include <type_traits>

namespace fluids {

// Minimal 2D vector. Deliberately a tight, standard-layout pair of floats
// (no padding, no virtuals) so a std::vector<Vec2> can be reinterpreted as a
// flat float buffer laid out [x0, y0, x1, y1, ...] with reinterpret_cast —
// this is how Simulation hands zero-copy position/velocity buffers to JS.
// The static_asserts below guard that assumption.
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2& operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vec2& operator*=(float s) {
        x *= s;
        y *= s;
        return *this;
    }

    float dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
    float lengthSquared() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSquared()); }
};

inline Vec2 operator+(Vec2 a, const Vec2& b) {
    a += b;
    return a;
}

inline Vec2 operator-(Vec2 a, const Vec2& b) {
    a -= b;
    return a;
}

inline Vec2 operator-(const Vec2& a) { return Vec2(-a.x, -a.y); }

inline Vec2 operator*(Vec2 a, float s) {
    a *= s;
    return a;
}

inline Vec2 operator*(float s, Vec2 a) {
    a *= s;
    return a;
}

static_assert(sizeof(Vec2) == sizeof(float) * 2,
              "Vec2 must be tightly packed (no padding) so it can be "
              "reinterpreted as a flat float array for the WASM<->JS "
              "zero-copy buffer contract.");
static_assert(std::is_standard_layout<Vec2>::value,
              "Vec2 must be standard-layout for reinterpret_cast<const "
              "float*> to be well-defined.");

} // namespace fluids
