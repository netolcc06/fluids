#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fluids/common/Vec3.h"

using fluids::Vec3;
using Catch::Approx;

TEST_CASE("Vec3 arithmetic", "[vec3]") {
    Vec3 a{1.0f, 2.0f, -3.0f};
    Vec3 b{3.0f, -1.0f, 2.0f};

    SECTION("add") {
        Vec3 c = a + b;
        REQUIRE(c.x == Approx(4.0f));
        REQUIRE(c.y == Approx(1.0f));
        REQUIRE(c.z == Approx(-1.0f));
    }

    SECTION("subtract") {
        Vec3 c = a - b;
        REQUIRE(c.x == Approx(-2.0f));
        REQUIRE(c.y == Approx(3.0f));
        REQUIRE(c.z == Approx(-5.0f));
    }

    SECTION("scale") {
        Vec3 c = a * 2.0f;
        REQUIRE(c.x == Approx(2.0f));
        REQUIRE(c.y == Approx(4.0f));
        REQUIRE(c.z == Approx(-6.0f));

        Vec3 d = 2.0f * a;
        REQUIRE(d.x == Approx(2.0f));
        REQUIRE(d.y == Approx(4.0f));
        REQUIRE(d.z == Approx(-6.0f));
    }

    SECTION("dot") { REQUIRE(a.dot(b) == Approx(1.0f * 3.0f + 2.0f * -1.0f + -3.0f * 2.0f)); }

    SECTION("length") {
        Vec3 c{2.0f, 3.0f, 6.0f};
        REQUIRE(c.length() == Approx(7.0f));
        REQUIRE(c.lengthSquared() == Approx(49.0f));
    }

    SECTION("compound assignment") {
        Vec3 c = a;
        c += b;
        REQUIRE(c.x == Approx(4.0f));
        REQUIRE(c.y == Approx(1.0f));
        REQUIRE(c.z == Approx(-1.0f));

        c -= b;
        REQUIRE(c.x == Approx(a.x));
        REQUIRE(c.y == Approx(a.y));
        REQUIRE(c.z == Approx(a.z));
    }

    SECTION("cross product") {
        Vec3 x{1.0f, 0.0f, 0.0f};
        Vec3 y{0.0f, 1.0f, 0.0f};
        Vec3 z = x.cross(y);
        // Right-handed system: x cross y = z (matches this project's 3D
        // coordinate convention — see Scenario3D.h).
        REQUIRE(z.x == Approx(0.0f));
        REQUIRE(z.y == Approx(0.0f));
        REQUIRE(z.z == Approx(1.0f));

        // Anticommutative.
        Vec3 negZ = y.cross(x);
        REQUIRE(negZ.z == Approx(-1.0f));

        // Cross of a vector with itself is zero.
        Vec3 zero = a.cross(a);
        REQUIRE(zero.length() == Approx(0.0f).margin(1e-6f));
    }
}
