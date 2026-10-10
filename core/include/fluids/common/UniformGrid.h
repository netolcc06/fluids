#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "fluids/common/Vec2.h"

namespace fluids {

// Spatial hash grid over a set of 2D points, used for O(1)-average neighbor
// queries in the SPH solver. Cell size is chosen by the caller (SPH uses the
// smoothing radius h, so any query with radius <= a few h only needs to scan
// a small ring of cells).
//
// Deviation from the master plan's `UniformGrid<T>` template: this is a
// concrete grid over `Vec2` points rather than a generic template. Phase 1
// only ever buckets particle positions, so genericity isn't exercised yet;
// templatizing later (for FLIP/MPM grid needs) won't break this class's
// callers since it's internal to core/solvers, not part of the JS contract.
//
// Usage: call build() once per solver step with the current positions, then
// call queryNeighbors() per particle. build() stores a pointer to the
// `points` vector passed in — the caller must keep it alive and not resize
// it between build() and any queryNeighbors() calls.
class UniformGrid {
public:
    explicit UniformGrid(float cellSize);

    void build(const std::vector<Vec2>& points);

    // Appends indices of points within `radius` of `origin` to `outIndices`
    // (does not clear outIndices first — caller controls that). Exact
    // distance filtering is applied internally (not just a coarse cell
    // superset), so results match a brute-force O(n) radius query.
    void queryNeighbors(const Vec2& origin, float radius, std::vector<int>& outIndices) const;

private:
    float cellSize_;
    const std::vector<Vec2>* points_ = nullptr;
    std::unordered_map<std::int64_t, std::vector<int>> cells_;

    std::pair<int, int> cellCoord(const Vec2& p) const;
    static std::int64_t makeKey(int ix, int iy);
};

} // namespace fluids
