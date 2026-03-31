#include "passable_area/core/preprocess/frame_preprocessor.hpp"

#include <Eigen/Geometry>

#include <cmath>
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

bool FramePreprocessor::process(const FrameInput &input, ProcessedFrame &output) const {
  output = ProcessedFrame{};
  output.stamp = input.stamp;
  output.base_pose_in_local = input.base_pose_in_local;
  output.processing_enabled = input.processing_enabled;
  if (!input.processing_enabled || input.input_cloud_in_base.empty()) {
    return false;
  }

  const Eigen::Affine3f transform =
      Eigen::Translation3f(input.base_pose_in_local.position) *
      input.base_pose_in_local.orientation.normalized();
  const float voxel_size = std::max(config_.preprocess.voxel_size, 1e-3f);
  std::unordered_map<VoxelKey, GravityPointSample, VoxelKeyHash> voxels;

  output.cloud_in_base.reserve(input.input_cloud_in_base.size());
  output.cloud_in_gravity.reserve(input.input_cloud_in_base.size());
  output.gravity_samples.reserve(input.input_cloud_in_base.size());

  for (const auto &raw_point : input.input_cloud_in_base) {
    if (!IsFinite(raw_point)) {
      continue;
    }
    output.cloud_in_base.push_back(raw_point);

    const Eigen::Vector3f body_point(raw_point.x, raw_point.y, raw_point.z);
    const Eigen::Vector3f gravity_point = transform * body_point;
    if (gravity_point.z() < config_.map.height_min || gravity_point.z() > config_.map.height_max) {
      continue;
    }

    const Point3f point_in_gravity{gravity_point.x(), gravity_point.y(), gravity_point.z()};
    if (!config_.preprocess.enable_downsample) {
      output.cloud_in_gravity.push_back(point_in_gravity);
      output.gravity_samples.push_back(GravityPointSample{raw_point, point_in_gravity});
      continue;
    }

    const VoxelKey key{
        static_cast<int>(std::floor(point_in_gravity.x / voxel_size)),
        static_cast<int>(std::floor(point_in_gravity.y / voxel_size)),
        static_cast<int>(std::floor(point_in_gravity.z / voxel_size)),
    };
    voxels.emplace(key, GravityPointSample{raw_point, point_in_gravity});
  }

  if (config_.preprocess.enable_downsample) {
    output.cloud_in_gravity.reserve(voxels.size());
    output.gravity_samples.reserve(voxels.size());
    for (const auto &entry : voxels) {
      output.cloud_in_gravity.push_back(entry.second.point_in_gravity);
      output.gravity_samples.push_back(entry.second);
    }
  }

  return !output.cloud_in_base.empty() && !output.cloud_in_gravity.empty();
}

} // namespace passable_area::core
