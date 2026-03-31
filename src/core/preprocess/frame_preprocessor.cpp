#include "passable_area/core/preprocess/frame_preprocessor.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <unordered_map>

namespace passable_area::core {
namespace {

struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey &key) const {
    size_t seed = 0;
    seed ^= std::hash<int>{}(key.x) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

bool IsFinite(const Point3f &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

} // namespace

bool FramePreprocessor::process(const FrameInput &input, FrameInput &output) const {
  output = FrameInput{};
  output.stamp = input.stamp;
  output.base_pose_in_local = input.base_pose_in_local;
  output.processing_enabled = input.processing_enabled;
  if (!input.processing_enabled || input.merged_cloud.empty()) {
    return false;
  }

  const float voxel_size = std::max(config_.preprocess.voxel_size, 1e-3f);
  std::unordered_map<VoxelKey, Point3f, VoxelKeyHash> voxels;
  output.merged_cloud.reserve(input.merged_cloud.size());

  const Eigen::Affine3f transform =
      Eigen::Translation3f(input.base_pose_in_local.position) *
      input.base_pose_in_local.orientation.normalized();

  for (const auto &raw_point : input.merged_cloud) {
    if (!IsFinite(raw_point)) {
      continue;
    }
    const Eigen::Vector3f body_point(raw_point.x, raw_point.y, raw_point.z);
    const Eigen::Vector3f local_point = transform * body_point;
    if (local_point.z() < config_.map.height_min || local_point.z() > config_.map.height_max) {
      continue;
    }

    Point3f point{local_point.x(), local_point.y(), local_point.z()};
    if (!config_.preprocess.enable_downsample) {
      output.merged_cloud.push_back(point);
      continue;
    }

    const VoxelKey key{
        static_cast<int>(std::floor(point.x / voxel_size)),
        static_cast<int>(std::floor(point.y / voxel_size)),
        static_cast<int>(std::floor(point.z / voxel_size)),
    };
    voxels.emplace(key, point);
  }

  if (config_.preprocess.enable_downsample) {
    output.merged_cloud.reserve(voxels.size());
    for (const auto &entry : voxels) {
      output.merged_cloud.push_back(entry.second);
    }
  }
  return !output.merged_cloud.empty();
}

} // namespace passable_area::core
