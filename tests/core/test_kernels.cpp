#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "fluids/common/Vec2.h"
#include "fluids/solvers/sph/Kernels.h"

using fluids::Vec2;
using fluids::sph::poly6;
using fluids::sph::spikyGradient;
using fluids::sph::viscosityLaplacian;
using Catch::Approx;

constexpr float kH = 0.1f;

TEST_CASE("poly6 kernel sanity", "[kernels][poly6]") {
    SECTION("maximal at r=0") {
        REQUIRE(poly6(0.0f, kH) > 0.0f);
        // poly6's effective r is floored at 0.5*h (see kMinEffectiveRFraction
        // in Kernels.h — a deliberate fix for the SPH "random launch" bug:
        // poly6 is nearly flat near r=0, so without a floor a single
        // near-coincident neighbor pair could inflate density almost to
        // poly6(0,h) and, via the Tait EOS's 7th-power response, produce a
        // wildly disproportionate pressure spike). That means the kernel
        // value is intentionally *constant* (not strictly decreasing) for
        // r in [0, 0.5*h] — r=0 and r=0.5*h tie for the max.
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

TEST_CASE("spiky gradient sanity", "[kernels][spiky]") {
    SECTION("zero vector at r~0 (undefined direction, safety clamp)") {
        Vec2 g = spikyGradient(Vec2{0.0f, 0.0f}, 0.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
    }

    SECTION("zero at r=h") {
        Vec2 rVec{kH, 0.0f};
        Vec2 g = spikyGradient(rVec, kH, kH);
        REQUIRE(g.length() == Approx(0.0f).margin(1e-6f));
    }

    SECTION("zero beyond r=h") {
        Vec2 rVec{kH * 2.0f, 0.0f};
        Vec2 g = spikyGradient(rVec, kH * 2.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
    }

    SECTION("nonzero within support, points along rVec's line, magnitude decreases toward h") {
        float r1 = kH * 0.25f;
        float r2 = kH * 0.75f;
        Vec2 g1 = spikyGradient(Vec2{r1, 0.0f}, r1, kH);
        Vec2 g2 = spikyGradient(Vec2{r2, 0.0f}, r2, kH);

        REQUIRE(g1.length() > 0.0f);
        REQUIRE(g2.length() > 0.0f);
        // Closer particles produce a stronger (repulsive) gradient.
        REQUIRE(g1.length() > g2.length());
        // Gradient of a radially-symmetric kernel along the x-axis has no
        // y-component.
        REQUIRE(g1.y == Approx(0.0f).margin(1e-6f));
    }
}

TEST_CASE("spiky gradient stays bounded and non-zero for very small but nonzero r", "[kernels][spiky][regression]") {
    // Regression coverage for the SPH "random launch" bug: the old code
    // returned a hard zero vector for any r < 1e-6, which let
    // near-coincident particle pairs feel no repulsion from each other at
    // all and drift into a permanent, silent overlap. The fix floors the
    // *effective* r used in the magnitude formula (not the direction) at
    // 0.5*h, so even a pair 1e-8 apart now gets a well-defined, bounded,
    // correctly-directed push apart.
    SECTION("nonzero for r far below the old 1e-6 cutoff") {
        Vec2 rVec{1e-8f, 0.0f};
        Vec2 g = spikyGradient(rVec, 1e-8f, kH);
        REQUIRE(g.length() > 0.0f);
        // The kernel gradient itself points opposite rVec (dW/dr < 0 for
        // the spiky kernel); computeForces negates it to make the actual
        // pressure force repulsive. Same sign convention as the existing
        // "nonzero within support" section above.
        REQUIRE(g.x < 0.0f);
    }

    SECTION("magnitude near r=0 is bounded, not unbounded, and no larger than at the floor") {
        Vec2 gAtTinyR = spikyGradient(Vec2{1e-8f, 0.0f}, 1e-8f, kH);
        Vec2 gAtFloor = spikyGradient(Vec2{kH * 0.5f, 0.0f}, kH * 0.5f, kH);
        REQUIRE(std::isfinite(gAtTinyR.length()));
        // Both r=1e-8 and r=0.5*h are floored to the same effective r, so
        // their magnitudes should match (within float precision) rather
        // than the tiny-r case blowing up.
        REQUIRE(gAtTinyR.length() == Approx(gAtFloor.length()).epsilon(0.01));
    }

    SECTION("exact coincidence (rVec = zero vector) still returns zero — direction is genuinely undefined") {
        Vec2 g = spikyGradient(Vec2{0.0f, 0.0f}, 0.0f, kH);
        REQUIRE(g.x == 0.0f);
        REQUIRE(g.y == 0.0f);
    }
}

TEST_CASE("viscosity laplacian sanity", "[kernels][viscosity]") {
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
