#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "fluids/common/Vec3.h"

namespace fluids {

// Spatial hash grid over a set of 3D points, the 3D sibling of UniformGrid
// (see UniformGrid.h). Used for O(1)-average neighbor queries in the 3D SPH
// solver; cell size is chosen by the caller (SPH uses the smoothing radius
// h). A radius query scans the surrounding 3x3x3 = 27 cells instead of the
// 2D grid's 3x3 = 9.
//
// Deviation from the master plan's generic `UniformGrid<T>`: like the 2D
// UniformGrid, this is concrete over `Vec3` rather than templated — the same
// YAGNI reasoning applies (see UniformGrid.h's own comment): only 3D SPH
// buckets particle positions with this so far, and templatizing later for
// other 3D solvers (should they ever need a differently-shaped grid) won't
// break this class's callers since it's internal to core/solvers, not part
// of the JS contract.
//
// Usage: call build() once per solver step with the current positions, then
// call queryNeighbors() per particle. build() stores a pointer to the
// `points` vector passed in — the caller must keep it alive and not resize
// it between build() and any queryNeighbors() calls.
class UniformGrid3D {
public:
    explicit UniformGrid3D(float cellSize);

    void build(const std::vector<Vec3>& points);

    // Appends indices of points within `radius` of `origin` to `outIndices`
    // (does not clear outIndices first — caller controls that). Exact
    // distance filtering is applied internally (not just a coarse cell
    // superset), so results match a brute-force O(n) radius query.
    void queryNeighbors(const Vec3& origin, float radius, std::vector<int>& outIndices) const;

private:
    // Unlike the 2D grid's bijective packing of two 32-bit cell coordinates
    // into a single int64 key, packing three 32-bit coordinates losslessly
    // into one 64-bit integer isn't possible. Using an explicit key struct
    // with a hash-combine (rather than a lossy pack) keeps this correct
    // regardless of hash quality: unordered_map resolves any hash collision
    // via operator==, so a suboptimal hash only costs performance, never
    // correctness.
    struct CellKey {
        std::int32_t ix = 0;
        std::int32_t iy = 0;
        std::int32_t iz = 0;

        bool operator==(const CellKey& other) const {
            return ix == other.ix && iy == other.iy && iz == other.iz;
        }
    };

    struct CellKeyHash {
        std::size_t operator()(const CellKey& k) const noexcept {
            std::size_t h = std::hash<std::int32_t>{}(k.ix);
            h ^= std::hash<std::int32_t>{}(k.iy) + 0x9e3779b9u + (h << 6) + (h >> 2);
            h ^= std::hash<std::int32_t>{}(k.iz) + 0x9e3779b9u + (h << 6) + (h >> 2);
            return h;
        }
    };

    float cellSize_;
    const std::vector<Vec3>* points_ = nullptr;
    std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells_;

    CellKey cellCoord(const Vec3& p) const;
};

} // namespace fluids
