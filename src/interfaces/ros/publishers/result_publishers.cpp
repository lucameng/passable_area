#include "passable_area/interfaces/ros/publishers/result_publishers.hpp"

namespace passable_area::interfaces::ros {

void ResultPublishers::initialize(rclcpp::Node &node) {
  terrain_state_pub_ =
      node.create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain_state", 10);
  terrain_cost_pub_ =
      node.create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain_cost", 10);
  grid_map_pub_ =
      node.create_publisher<grid_map_msgs::msg::GridMap>("/terrain_debug/grid_map", 10);
}

void ResultPublishers::publish(const passable_area::core::FrameOutput &output,
                               const std_msgs::msg::Header &header) {
  terrain_state_pub_->publish(converter_.toTerrainState(output, header));
  terrain_cost_pub_->publish(converter_.toTerrainCost(output, header));
  if (grid_map_pub_) {
    grid_map_pub_->publish(converter_.toGridMap(output, header));
  }
}

} // namespace passable_area::interfaces::ros
