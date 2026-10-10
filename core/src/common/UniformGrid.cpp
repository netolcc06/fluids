#include "fluids/common/UniformGrid.h"

#include <cmath>

namespace fluids {

UniformGrid::UniformGrid(float cellSize) : cellSize_(cellSize) {}

std::pair<int, int> UniformGrid::cellCoord(const Vec2& p) const {
    // floor (not truncation) so negative coordinates bucket correctly.
    int ix = static_cast<int>(std::floor(p.x / cellSize_));
    int iy = static_cast<int>(std::floor(p.y / cellSize_));
    return {ix, iy};
}

std::int64_t UniformGrid::makeKey(int ix, int iy) {
    // Pack two 32-bit (possibly negative) cell coordinates into a 64-bit
    // key. Reinterpreting each as its raw uint32 bit pattern before packing
    // keeps the mapping a bijection (no collisions) for the full int32
    // range in each axis.
    return (static_cast<std::int64_t>(static_cast<std::uint32_t>(ix)) << 32) |
           static_cast<std::uint32_t>(iy);
}

void UniformGrid::build(const std::vector<Vec2>& points) {
    points_ = &points;
    cells_.clear();
    cells_.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        auto [ix, iy] = cellCoord(points[i]);
        cells_[makeKey(ix, iy)].push_back(static_cast<int>(i));
    }
}

void UniformGrid::queryNeighbors(const Vec2& origin, float radius, std::vector<int>& outIndices) const {
    if (points_ == nullptr) return;

    const auto [cx, cy] = cellCoord(origin);
    const int range = static_cast<int>(std::ceil(radius / cellSize_));
    const float r2 = radius * radius;

    for (int dy = -range; dy <= range; ++dy) {
        for (int dx = -range; dx <= range; ++dx) {
            auto it = cells_.find(makeKey(cx + dx, cy + dy));
            if (it == cells_.end()) continue;
            for (int idx : it->second) {
                const Vec2 diff = (*points_)[idx] - origin;
                if (diff.lengthSquared() <= r2) {
                    outIndices.push_back(idx);
                }
            }
        }
    }
}

} // namespace fluids
