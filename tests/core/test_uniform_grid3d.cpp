#include <algorithm>
#include <random>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "fluids/common/UniformGrid3D.h"
#include "fluids/common/Vec3.h"

using fluids::UniformGrid3D;
using fluids::Vec3;

namespace {

std::vector<int> bruteForceNeighbors(const std::vector<Vec3>& points, const Vec3& origin, float radius) {
    std::vector<int> result;
    const float r2 = radius * radius;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if ((points[i] - origin).lengthSquared() <= r2) {
            result.push_back(static_cast<int>(i));
        }
    }
    return result;
}

} // namespace

TEST_CASE("UniformGrid3D matches brute-force neighbor search", "[uniform_grid3d]") {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> coord(-2.0f, 2.0f);

    const int kNumPoints = 200;
    std::vector<Vec3> points;
    points.reserve(kNumPoints);
    for (int i = 0; i < kNumPoints; ++i) {
        points.emplace_back(coord(rng), coord(rng), coord(rng));
    }

    const float cellSize = 0.2f;
    UniformGrid3D grid(cellSize);
    grid.build(points);

    const float radii[] = {0.05f, cellSize, cellSize * 2.5f};

    for (float radius : radii) {
        for (int qi = 0; qi < 25; ++qi) {
            Vec3 origin{coord(rng), coord(rng), coord(rng)};

            std::vector<int> expected = bruteForceNeighbors(points, origin, radius);
            std::vector<int> actual;
            grid.queryNeighbors(origin, radius, actual);

            std::sort(expected.begin(), expected.end());
            std::sort(actual.begin(), actual.end());

            REQUIRE(actual == expected);
        }
    }
}

TEST_CASE("UniformGrid3D query includes the point itself at r=0", "[uniform_grid3d]") {
    std::vector<Vec3> points = {{0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, {0.01f, 0.0f, 0.0f}};
    UniformGrid3D grid(0.1f);
    grid.build(points);

    std::vector<int> result;
    grid.queryNeighbors(points[0], 0.1f, result);

    std::sort(result.begin(), result.end());
    REQUIRE(result == std::vector<int>{0, 2});
}

TEST_CASE("UniformGrid3D handles empty point sets", "[uniform_grid3d]") {
    std::vector<Vec3> points;
    UniformGrid3D grid(0.1f);
    grid.build(points);

    std::vector<int> result;
    grid.queryNeighbors(Vec3{0.0f, 0.0f, 0.0f}, 1.0f, result);
    REQUIRE(result.empty());
}

TEST_CASE("UniformGrid3D handles negative coordinates", "[uniform_grid3d]") {
    std::vector<Vec3> points = {{-5.5f, -5.5f, -5.5f}, {-5.45f, -5.5f, -5.5f}, {5.0f, 5.0f, 5.0f}};
    UniformGrid3D grid(0.2f);
    grid.build(points);

    std::vector<int> result;
    grid.queryNeighbors(points[0], 0.2f, result);
    std::sort(result.begin(), result.end());
    REQUIRE(result == std::vector<int>{0, 1});
}
