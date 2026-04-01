#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"

namespace passable_area::interfaces::ros {

RosNodeParams RosParamLoader::load(rclcpp::Node &node) const {
  RosNodeParams params;
  auto &config = params.config;
  auto &topics = params.topics;

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
  config.preprocess.body_filter.enable =
      node.declare_parameter("preprocess.body_filter.enable", config.preprocess.body_filter.enable);
  config.preprocess.body_filter.x_min =
      node.declare_parameter("preprocess.body_filter.x_min", config.preprocess.body_filter.x_min);
  config.preprocess.body_filter.x_max =
      node.declare_parameter("preprocess.body_filter.x_max", config.preprocess.body_filter.x_max);
  config.preprocess.body_filter.y_min =
      node.declare_parameter("preprocess.body_filter.y_min", config.preprocess.body_filter.y_min);
  config.preprocess.body_filter.y_max =
      node.declare_parameter("preprocess.body_filter.y_max", config.preprocess.body_filter.y_max);
  config.preprocess.body_filter.z_min =
      node.declare_parameter("preprocess.body_filter.z_min", config.preprocess.body_filter.z_min);
  config.preprocess.body_filter.z_max =
      node.declare_parameter("preprocess.body_filter.z_max", config.preprocess.body_filter.z_max);
  config.preprocess.crop_to_map.enable =
      node.declare_parameter("preprocess.crop_to_map.enable", config.preprocess.crop_to_map.enable);
  config.preprocess.crop_to_map.xy_margin = node.declare_parameter(
      "preprocess.crop_to_map.xy_margin", config.preprocess.crop_to_map.xy_margin);

  config.odom_frame = node.declare_parameter("odom_frame", config.odom_frame);
  config.base_gravity_frame =
      node.declare_parameter("base_gravity_frame", config.base_gravity_frame);
  config.body_frame = node.declare_parameter("body_frame", config.body_frame);
  config.debug.publish_grid_map =
      node.declare_parameter("debug.publish_grid_map", config.debug.publish_grid_map);
  config.debug.publish_points =
      node.declare_parameter("debug.publish_points", config.debug.publish_points);
  config.debug.publish_base_gravity_cloud = node.declare_parameter(
      "debug.publish_base_gravity_cloud", config.debug.publish_base_gravity_cloud);
  config.debug.publish_observability =
      node.declare_parameter("debug.publish_observability", config.debug.publish_observability);
  topics.input_cloud_topic =
      node.declare_parameter("input_cloud_topic", topics.input_cloud_topic);
  topics.odom_topic = node.declare_parameter("odom_topic", topics.odom_topic);
  topics.sync_queue_size = node.declare_parameter("sync.queue_size", topics.sync_queue_size);
  topics.terrain_state_topic =
      node.declare_parameter("output.terrain_state_topic", topics.terrain_state_topic);
  topics.terrain_cost_topic =
      node.declare_parameter("output.terrain_cost_topic", topics.terrain_cost_topic);
  topics.debug_grid_map_topic =
      node.declare_parameter("output.debug_grid_map_topic", topics.debug_grid_map_topic);
  topics.base_gravity_cloud_topic = node.declare_parameter("output.base_gravity_cloud_topic",
                                                           topics.base_gravity_cloud_topic);
  topics.support_points_topic =
      node.declare_parameter("output.support_points_topic", topics.support_points_topic);
  topics.obstacle_points_topic =
      node.declare_parameter("output.obstacle_points_topic", topics.obstacle_points_topic);
  topics.unknown_mask_topic =
      node.declare_parameter("output.unknown_mask_topic", topics.unknown_mask_topic);
  topics.observability_topic =
      node.declare_parameter("output.observability_topic", topics.observability_topic);
  return params;
}

} // namespace passable_area::interfaces::ros
