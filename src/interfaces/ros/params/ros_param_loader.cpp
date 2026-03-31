#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"

namespace passable_area::interfaces::ros {

passable_area::core::Config RosParamLoader::load(rclcpp::Node &node) const {
  passable_area::core::Config config;
  config.map.length = node.declare_parameter("map_length", config.map.length);
  config.map.width = node.declare_parameter("map_width", config.map.width);
  config.map.resolution = node.declare_parameter("map_resolution", config.map.resolution);
  config.map.height_min = node.declare_parameter("map_height_min", config.map.height_min);
  config.map.height_max = node.declare_parameter("map_height_max", config.map.height_max);

  config.geometry.max_support_slope_deg =
      node.declare_parameter("max_support_slope", config.geometry.max_support_slope_deg);
  config.geometry.max_step_up = node.declare_parameter("max_step_up", config.geometry.max_step_up);
  config.geometry.max_step_down =
      node.declare_parameter("max_step_down", config.geometry.max_step_down);
  config.geometry.max_support_roughness = node.declare_parameter(
      "max_support_roughness", config.geometry.max_support_roughness);
  config.geometry.min_clearance =
      node.declare_parameter("min_clearance", config.geometry.min_clearance);

  config.observability.dropout_sector_gap_threshold = node.declare_parameter(
      "dropout_sector_gap_threshold", config.observability.dropout_sector_gap_threshold);
  config.observability.stale_to_unknown_time =
      node.declare_parameter("stale_to_unknown_time", config.observability.stale_to_unknown_time);
  config.observability.min_support_confidence = node.declare_parameter(
      "min_support_confidence", config.observability.min_support_confidence);
  config.observability.sector_count =
      node.declare_parameter("sector_count", config.observability.sector_count);
  config.observability.min_points_per_sector = node.declare_parameter(
      "min_points_per_sector", config.observability.min_points_per_sector);

  config.persistence.support_persistence_frames = node.declare_parameter(
      "support_persistence_frames", config.persistence.support_persistence_frames);
  config.preprocess.enable_downsample =
      node.declare_parameter("downsample.enable", config.preprocess.enable_downsample);
  config.preprocess.voxel_size =
      node.declare_parameter("downsample.voxel_size", config.preprocess.voxel_size);

  config.world_frame = node.declare_parameter("world_frame", config.world_frame);
  config.gravity_frame = node.declare_parameter("gravity_frame", config.gravity_frame);
  config.body_frame = node.declare_parameter("body_frame", config.body_frame);
  config.debug.publish_grid_map =
      node.declare_parameter("debug.publish_grid_map", config.debug.publish_grid_map);
  config.debug.publish_points =
      node.declare_parameter("debug.publish_points", config.debug.publish_points);
  config.debug.publish_observability =
      node.declare_parameter("debug.publish_observability", config.debug.publish_observability);
  return config;
}

} // namespace passable_area::interfaces::ros
