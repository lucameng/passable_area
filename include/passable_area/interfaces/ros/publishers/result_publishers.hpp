#ifndef PASSABLE_AREA_INTERFACES_ROS_PUBLISHERS_RESULT_PUBLISHERS_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_PUBLISHERS_RESULT_PUBLISHERS_HPP_

#include "passable_area/core/types/frame_types.hpp"
#include "passable_area/interfaces/ros/converters/output_converter.hpp"
#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"

#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace passable_area::interfaces::ros {

class ResultPublishers {
public:
  void initialize(rclcpp::Node &node, const RosTopicConfig &topics);
  void publish(const passable_area::core::FrameOutput &output, const std_msgs::msg::Header &header);

private:
  OutputConverter converter_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_cost_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
};

} // namespace passable_area::interfaces::ros

#endif
