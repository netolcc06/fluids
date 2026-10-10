#include "fluids/common/UniformGrid3D.h"

#include <cmath>

namespace fluids {

UniformGrid3D::UniformGrid3D(float cellSize) : cellSize_(cellSize) {}

UniformGrid3D::CellKey UniformGrid3D::cellCoord(const Vec3& p) const {
    // floor (not truncation) so negative coordinates bucket correctly.
    CellKey key;
    key.ix = static_cast<std::int32_t>(std::floor(p.x / cellSize_));
    key.iy = static_cast<std::int32_t>(std::floor(p.y / cellSize_));
    key.iz = static_cast<std::int32_t>(std::floor(p.z / cellSize_));
    return key;
}

void UniformGrid3D::build(const std::vector<Vec3>& points) {
    points_ = &points;
    cells_.clear();
    cells_.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        cells_[cellCoord(points[i])].push_back(static_cast<int>(i));
    }
}

void UniformGrid3D::queryNeighbors(const Vec3& origin, float radius, std::vector<int>& outIndices) const {
    if (points_ == nullptr) return;

    const CellKey center = cellCoord(origin);
    const int range = static_cast<int>(std::ceil(radius / cellSize_));
    const float r2 = radius * radius;

    for (int dz = -range; dz <= range; ++dz) {
        for (int dy = -range; dy <= range; ++dy) {
            for (int dx = -range; dx <= range; ++dx) {
                CellKey key{center.ix + dx, center.iy + dy, center.iz + dz};
                auto it = cells_.find(key);
                if (it == cells_.end()) continue;
                for (int idx : it->second) {
                    const Vec3 diff = (*points_)[idx] - origin;
                    if (diff.lengthSquared() <= r2) {
                        outIndices.push_back(idx);
                    }
                }
            }
        }
    }
}

} // namespace fluids
