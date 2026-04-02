#ifndef PASSABLE_AREA_INTERFACES_ROS_PARAMS_ROS_PARAM_LOADER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_PARAMS_ROS_PARAM_LOADER_HPP_

#include "passable_area/core/types/config_types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace passable_area::interfaces::ros {

struct RosTopicConfig {
  std::string input_cloud_topic = "/LOC_BODY_POINTS";
  std::string odom_topic = "/ODOM";
  int sync_queue_size = 10;
  std::string terrain_state_topic = "/terrain_state";
  std::string terrain_cost_topic = "/terrain_cost";
  std::string debug_grid_map_topic = "/terrain_debug/grid_map";
  std::string base_gravity_cloud_topic = "/terrain_debug/base_gravity_cloud";
  std::string support_points_topic = "/terrain_debug/support_points";
  std::string obstacle_points_topic = "/terrain_obstacle_points";
  std::string unknown_mask_topic = "/terrain_debug/unknown_mask";
  std::string observability_topic = "/terrain_debug/observability";
};

struct RosNodeParams {
  passable_area::core::Config config;
  RosTopicConfig topics;
};

class RosParamLoader {
public:
  RosNodeParams load(rclcpp::Node &node) const;
};

} // namespace passable_area::interfaces::ros

#endif
