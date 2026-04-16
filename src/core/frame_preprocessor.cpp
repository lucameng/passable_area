#include "passable_area/core/frame_preprocessor.hpp"

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
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool IsInsideBodyFilter(const Point3f &point,
                        const PreprocessConfig::BodyFilterConfig &config) {
  return point.x >= config.x_min && point.x <= config.x_max &&
         point.y >= config.y_min && point.y <= config.y_max &&
         point.z >= config.z_min && point.z <= config.z_max;
}

bool IsInsideCropWindow(const Eigen::Vector3f &point_in_map,
                        const Pose3D &base_pose_in_map, const Config &config,
                        float voxel_size) {
  if (!config.preprocess.crop_to_map.enable) {
    return true;
  }

  const float xy_margin = std::max(config.preprocess.crop_to_map.xy_margin,
                                   std::max(config.map.resolution, voxel_size));
  const float half_length = 0.5f * config.map.length + xy_margin;
  const float half_width = 0.5f * config.map.width + xy_margin;
  const float dx = point_in_map.x() - base_pose_in_map.position.x();
  const float dy = point_in_map.y() - base_pose_in_map.position.y();
  if (std::abs(dx) > half_length || std::abs(dy) > half_width) {
    return false;
  }

  const float relative_z = point_in_map.z() - base_pose_in_map.position.z();
  return relative_z >= config.map.height_min &&
         relative_z <= config.map.height_max;
}

} // namespace

bool FramePreprocessor::process(const FrameInput &input,
                                ProcessedFrame &output) const {
  output = ProcessedFrame{};
  output.stamp = input.stamp;
  output.base_pose_in_map = input.base_pose_in_map;
  output.processing_enabled = input.processing_enabled;
  if (!input.processing_enabled || input.input_cloud_in_base.empty()) {
    return false;
  }

  const Eigen::Affine3f transform =
      Eigen::Translation3f(input.base_pose_in_map.position) *
      input.base_pose_in_map.orientation.normalized();
  const float voxel_size = std::max(config_.preprocess.voxel_size, 1e-3f);
  std::unordered_map<VoxelKey, MapPointSample, VoxelKeyHash> voxels;

  output.cloud_in_base.reserve(input.input_cloud_in_base.size());
  output.cloud_in_map.reserve(input.input_cloud_in_base.size());
  output.map_samples.reserve(input.input_cloud_in_base.size());

  for (const auto &raw_point : input.input_cloud_in_base) {
    if (!IsFinite(raw_point)) {
      continue;
    }
    if (config_.preprocess.body_filter.enable &&
        IsInsideBodyFilter(raw_point, config_.preprocess.body_filter)) {
      continue;
    }

    const Eigen::Vector3f body_point(raw_point.x, raw_point.y, raw_point.z);
    const Eigen::Vector3f map_point = transform * body_point;
    if (!IsInsideCropWindow(map_point, input.base_pose_in_map, config_,
                            voxel_size)) {
      continue;
    }

    output.cloud_in_base.push_back(raw_point);
    const Point3f point_in_map{map_point.x(), map_point.y(), map_point.z()};
    if (!config_.preprocess.enable_downsample) {
      output.cloud_in_map.push_back(point_in_map);
      output.map_samples.push_back(MapPointSample{raw_point, point_in_map});
      continue;
    }

    const VoxelKey key{
        static_cast<int>(std::floor(point_in_map.x / voxel_size)),
        static_cast<int>(std::floor(point_in_map.y / voxel_size)),
        static_cast<int>(std::floor(point_in_map.z / voxel_size)),
    };
    voxels.emplace(key, MapPointSample{raw_point, point_in_map});
  }

  if (config_.preprocess.enable_downsample) {
    output.cloud_in_map.reserve(voxels.size());
    output.map_samples.reserve(voxels.size());
    for (const auto &entry : voxels) {
      output.cloud_in_map.push_back(entry.second.point_in_map);
      output.map_samples.push_back(entry.second);
    }
  }

  return !output.cloud_in_base.empty() && !output.cloud_in_map.empty();
}

} // namespace passable_area::core
