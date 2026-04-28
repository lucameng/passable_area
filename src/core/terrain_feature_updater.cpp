#include "passable_area/core/terrain_feature_updater.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace passable_area::core {
namespace {

struct SurfacePoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

bool IsSupportGeometryEligible(const TerrainLayers &layers, int cell) {
  return layers.support_surface_contaminated[static_cast<size_t>(cell)] == 0U;
}

bool FitSupportPlane(const std::array<SurfacePoint, 9> &points, int count,
                     Eigen::Vector3f &plane) {
  if (count < 3) {
    return false;
  }

  Eigen::Matrix3f normal = Eigen::Matrix3f::Zero();
  Eigen::Vector3f rhs = Eigen::Vector3f::Zero();
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3f row(points[static_cast<size_t>(i)].x,
                              points[static_cast<size_t>(i)].y, 1.0f);
    normal.noalias() += row * row.transpose();
    rhs.noalias() += row * points[static_cast<size_t>(i)].z;
  }

  if (std::abs(normal.determinant()) <= 1e-6f) {
    return false;
  }
  plane = normal.ldlt().solve(rhs);
  return plane.allFinite();
}

float PlaneRoughness(const std::array<SurfacePoint, 9> &points, int count,
                     const Eigen::Vector3f &plane) {
  if (count <= 0) {
    return 0.0f;
  }

  float residual_sq_sum = 0.0f;
  for (int i = 0; i < count; ++i) {
    const auto &point = points[static_cast<size_t>(i)];
    const float predicted_z = plane.x() * point.x + plane.y() * point.y +
                              plane.z();
    const float residual = point.z - predicted_z;
    residual_sq_sum += residual * residual;
  }
  return std::sqrt(residual_sq_sum / static_cast<float>(count));
}

} // namespace

void TerrainFeatureUpdater::update(const std::vector<int> &dirty_cells,
                                   LocalTerrainMap &map) const {
  auto &layers = map.layers();
  std::unordered_set<int> expanded;
  for (const int cell : dirty_cells) {
    expanded.insert(cell);
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        expanded.insert(nr * map.cols() + nc);
      }
    }
  }

  for (const int cell : expanded) {
    const float h = layers.support_height[cell];
    if (!std::isfinite(h)) {
      layers.slope[cell] = 0.0f;
      layers.step_up[cell] = 0.0f;
      layers.step_down[cell] = 0.0f;
      layers.roughness[cell] = 0.0f;
      layers.clearance[cell] = std::numeric_limits<float>::quiet_NaN();
      layers.support_continuity[cell] = 0.0f;
      continue;
    }

    float step_up = 0.0f;
    float step_down = 0.0f;
    float max_abs_grade = 0.0f;
    float rough_sum = 0.0f;
    std::array<SurfacePoint, 9> surface_points;
    int surface_point_count = 0;
    surface_points[static_cast<size_t>(surface_point_count++)] =
        SurfacePoint{0.0f, 0.0f, h};
    int valid_neighbors = 0;
    const int row = cell / map.cols();
    const int col = cell % map.cols();
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) {
          continue;
        }
        const int nr = row + dr;
        const int nc = col + dc;
        if (nr < 0 || nr >= map.rows() || nc < 0 || nc >= map.cols()) {
          continue;
        }
        const int neighbor = nr * map.cols() + nc;
        const float nh = layers.support_height[neighbor];
        if (!std::isfinite(nh) ||
            !IsSupportGeometryEligible(layers, neighbor)) {
          continue;
        }
        const float dz = nh - h;
        step_up = std::max(step_up, -dz);
        step_down = std::max(step_down, dz);
        const float neighbor_distance =
            map.resolution() * std::hypot(static_cast<float>(dr),
                                           static_cast<float>(dc));
        max_abs_grade =
            std::max(max_abs_grade,
                     std::abs(dz) / std::max(neighbor_distance, 1e-3f));
        rough_sum += dz * dz;
        surface_points[static_cast<size_t>(surface_point_count++)] =
            SurfacePoint{static_cast<float>(dc) * map.resolution(),
                         static_cast<float>(dr) * map.resolution(), nh};
        ++valid_neighbors;
      }
    }
    layers.step_up[cell] = step_up;
    layers.step_down[cell] = step_down;
    Eigen::Vector3f support_plane = Eigen::Vector3f::Zero();
    if (FitSupportPlane(surface_points, surface_point_count, support_plane)) {
      layers.slope[cell] =
          std::atan(std::hypot(support_plane.x(), support_plane.y())) *
          180.0f / static_cast<float>(M_PI);
      layers.roughness[cell] =
          PlaneRoughness(surface_points, surface_point_count, support_plane);
    } else {
      layers.slope[cell] = std::atan(max_abs_grade) *
                           180.0f / static_cast<float>(M_PI);
      layers.roughness[cell] =
          valid_neighbors > 0
              ? std::sqrt(rough_sum / static_cast<float>(valid_neighbors))
              : 0.0f;
    }
    layers.clearance[cell] = std::isfinite(layers.overhead_height[cell])
                                 ? layers.overhead_height[cell] - h
                                 : std::numeric_limits<float>::infinity();
    layers.support_continuity[cell] = static_cast<float>(valid_neighbors) /
                                      8.0f * layers.support_confidence[cell];
  }
}

} // namespace passable_area::core
