#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fluids/common/Vec2.h"

using fluids::Vec2;
using Catch::Approx;

TEST_CASE("Vec2 arithmetic", "[vec2]") {
    Vec2 a{1.0f, 2.0f};
    Vec2 b{3.0f, -1.0f};

    SECTION("add") {
        Vec2 c = a + b;
        REQUIRE(c.x == Approx(4.0f));
        REQUIRE(c.y == Approx(1.0f));
    }

    SECTION("subtract") {
        Vec2 c = a - b;
        REQUIRE(c.x == Approx(-2.0f));
        REQUIRE(c.y == Approx(3.0f));
    }

    SECTION("scale") {
        Vec2 c = a * 2.0f;
        REQUIRE(c.x == Approx(2.0f));
        REQUIRE(c.y == Approx(4.0f));

        Vec2 d = 2.0f * a;
        REQUIRE(d.x == Approx(2.0f));
        REQUIRE(d.y == Approx(4.0f));
    }

    SECTION("dot") { REQUIRE(a.dot(b) == Approx(1.0f)); }

    SECTION("length") {
        Vec2 c{3.0f, 4.0f};
        REQUIRE(c.length() == Approx(5.0f));
        REQUIRE(c.lengthSquared() == Approx(25.0f));
    }

    SECTION("compound assignment") {
        Vec2 c = a;
        c += b;
        REQUIRE(c.x == Approx(4.0f));
        REQUIRE(c.y == Approx(1.0f));

        c -= b;
        REQUIRE(c.x == Approx(a.x));
        REQUIRE(c.y == Approx(a.y));
    }
}
