#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fluids/common/Vec3.h"
#include "fluids/solvers/sph3d/Kernels3D.h"

using fluids::Vec3;
using fluids::sph3d::poly6;
using fluids::sph3d::spikyGradient;
using fluids::sph3d::viscosityLaplacian;
using Catch::Approx;

constexpr float kH = 0.1f;

TEST_CASE("3D poly6 kernel sanity", "[kernels3d][poly6]") {
    SECTION("maximal at r=0") {
        REQUIRE(poly6(0.0f, kH) > 0.0f);
        // Same near-coincidence floor as the 2D solver (see
        // kMinEffectiveRFraction in Kernels3D.h — baked in from day one to
        // avoid rediscovering the 2D SPH "random launch" bug): the kernel
        // value is intentionally *constant* for r in [0, 0.5*h].
        REQUIRE(poly6(0.0f, kH) == Approx(poly6(kH * 0.5f, kH)));
        // Strictly past the floor, it resumes strictly decreasing.
        REQUIRE(poly6(0.0f, kH) > poly6(kH * 0.75f, kH));
    }

    SECTION("zero at r=h") { REQUIRE(poly6(kH, kH) == Approx(0.0f).margin(1e-6f)); }

    SECTION("zero beyond r=h") {
        REQUIRE(poly6(kH * 1.01f, kH) == 0.0f);
        REQUIRE(poly6(kH * 10.0f, kH) == 0.0f);
    }

    SECTION("monotonically decreasing over [0, h]") {
        float prev = poly6(0.0f, kH);
        for (int i = 1; i <= 10; ++i) {
            float r = kH * static_cast<float>(i) / 10.0f;
            float v = poly6(r, kH);
            REQUIRE(v <= prev);
            prev = v;
        }
    }

    SECTION("negative r is treated as invalid (returns 0)") { REQUIRE(poly6(-1.0f, kH) == 0.0f); }
}

TEST_CASE("3D spiky gradient sanity", "[kernels3d][spiky]") {
    SECTION("zero vector at r~0 (undefined direction, safety clamp)") {
        Vec3 g = spikyGradient(Vec3{0.0f, 0.0f, 0.0f}, 0.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
        REQUIRE(g.z == 0.0f);
    }

    SECTION("zero at r=h") {
        Vec3 rVec{kH, 0.0f, 0.0f};
        Vec3 g = spikyGradient(rVec, kH, kH);
        REQUIRE(g.length() == Approx(0.0f).margin(1e-6f));
    }

    SECTION("zero beyond r=h") {
        Vec3 rVec{kH * 2.0f, 0.0f, 0.0f};
        Vec3 g = spikyGradient(rVec, kH * 2.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
        REQUIRE(g.z == 0.0f);
    }

    SECTION("nonzero within support, points along rVec's line, magnitude decreases toward h") {
        float r1 = kH * 0.25f;
        float r2 = kH * 0.75f;
        Vec3 g1 = spikyGradient(Vec3{r1, 0.0f, 0.0f}, r1, kH);
        Vec3 g2 = spikyGradient(Vec3{r2, 0.0f, 0.0f}, r2, kH);

        REQUIRE(g1.length() > 0.0f);
        REQUIRE(g2.length() > 0.0f);
        // Closer particles produce a stronger (repulsive) gradient.
        REQUIRE(g1.length() > g2.length());
        // Gradient of a radially-symmetric kernel along the x-axis has no
        // y or z component.
        REQUIRE(g1.y == Approx(0.0f).margin(1e-6f));
        REQUIRE(g1.z == Approx(0.0f).margin(1e-6f));
    }
}

TEST_CASE("3D spiky gradient stays bounded and non-zero for very small but nonzero r",
          "[kernels3d][spiky][regression]") {
    // Regression coverage baked in from day one for the 3D solver (mirroring
    // the 2D SPH "random launch" bug fix — see Kernels3D.h's
    // kMinEffectiveRFraction comment): a pair 1e-8 apart must still get a
    // well-defined, bounded, correctly-directed push apart, not a hard zero.
    SECTION("nonzero for r far below the old-style 1e-6 cutoff") {
        Vec3 rVec{1e-8f, 0.0f, 0.0f};
        Vec3 g = spikyGradient(rVec, 1e-8f, kH);
        REQUIRE(g.length() > 0.0f);
        REQUIRE(g.x < 0.0f);
    }

    SECTION("magnitude near r=0 is bounded, not unbounded, and matches the value at the floor") {
        Vec3 gAtTinyR = spikyGradient(Vec3{1e-8f, 0.0f, 0.0f}, 1e-8f, kH);
        Vec3 gAtFloor = spikyGradient(Vec3{kH * 0.5f, 0.0f, 0.0f}, kH * 0.5f, kH);
        REQUIRE(std::isfinite(gAtTinyR.length()));
        REQUIRE(gAtTinyR.length() == Approx(gAtFloor.length()).epsilon(0.01));
    }

    SECTION("exact coincidence (rVec = zero vector) still returns zero — direction is genuinely undefined") {
        Vec3 g = spikyGradient(Vec3{0.0f, 0.0f, 0.0f}, 0.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
        REQUIRE(g.z == 0.0f);
    }
}

TEST_CASE("3D viscosity laplacian sanity", "[kernels3d][viscosity]") {
    SECTION("maximal at r=0") {
        REQUIRE(viscosityLaplacian(0.0f, kH) > 0.0f);
        REQUIRE(viscosityLaplacian(0.0f, kH) > viscosityLaplacian(kH * 0.5f, kH));
    }

    SECTION("zero at r=h") { REQUIRE(viscosityLaplacian(kH, kH) == Approx(0.0f).margin(1e-6f)); }

    SECTION("zero beyond r=h") { REQUIRE(viscosityLaplacian(kH * 2.0f, kH) == 0.0f); }

    SECTION("linearly decreasing over [0, h]") {
        float prev = viscosityLaplacian(0.0f, kH);
        for (int i = 1; i <= 10; ++i) {
            float r = kH * static_cast<float>(i) / 10.0f;
            float v = viscosityLaplacian(r, kH);
            REQUIRE(v <= prev);
            prev = v;
        }
    }
}
