#pragma once

#include <cmath>
#include <type_traits>

namespace fluids {

// Minimal 3D vector, the 3D sibling of Vec2 (see Vec2.h — same reasoning
// applies here verbatim). Deliberately a tight, standard-layout triple of
// floats (no padding, no virtuals) so a std::vector<Vec3> can be
// reinterpreted as a flat float buffer laid out [x0, y0, z0, x1, y1, z1, ...]
// with reinterpret_cast — this is how SphSimulation3D hands zero-copy
// position/velocity buffers to JS (count * 3 floats, NOT count * 2 like every
// 2D solver — see docs/sph3d-sim-api.md). The static_asserts below guard
// that assumption.
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3& operator+=(const Vec3& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3& operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    float dot(const Vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    float lengthSquared() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSquared()); }

    // 3D cross product. Not needed by SPH itself (no torque/vorticity term in
    // this solver), but included per the master plan's ask since it's a
    // one-liner and a natural "same operations as Vec2, plus what's genuinely
    // new in 3D" addition — cheap to have, costs nothing if unused.
    Vec3 cross(const Vec3& rhs) const {
        return Vec3{y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }
};

inline Vec3 operator+(Vec3 a, const Vec3& b) {
    a += b;
    return a;
}

inline Vec3 operator-(Vec3 a, const Vec3& b) {
    a -= b;
    return a;
}

inline Vec3 operator-(const Vec3& a) { return Vec3(-a.x, -a.y, -a.z); }

inline Vec3 operator*(Vec3 a, float s) {
    a *= s;
    return a;
}

inline Vec3 operator*(float s, Vec3 a) {
    a *= s;
    return a;
}

static_assert(sizeof(Vec3) == sizeof(float) * 3,
              "Vec3 must be tightly packed (no padding) so it can be "
              "reinterpreted as a flat float array for the WASM<->JS "
              "zero-copy buffer contract.");
static_assert(std::is_standard_layout<Vec3>::value,
              "Vec3 must be standard-layout for reinterpret_cast<const "
              "float*> to be well-defined.");

} // namespace fluids
