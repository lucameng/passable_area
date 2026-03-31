#include "passable_area/interfaces/ros/publishers/debug_publishers.hpp"

namespace passable_area::interfaces::ros {

void DebugPublishers::initialize(rclcpp::Node &node) {
  support_points_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/terrain_debug/support_points", 10);
  obstacle_points_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/terrain_debug/obstacle_points", 10);
  unknown_mask_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/terrain_debug/unknown_mask", 10);
  observability_pub_ = node.create_publisher<passable_area::msg::TerrainObservability>(
      "/terrain_debug/observability", 10);
}

void DebugPublishers::publish(const passable_area::core::FrameOutput &output,
                              const std_msgs::msg::Header &header) {
  support_points_pub_->publish(point_converter_.toRos(output.support_points, header));
  obstacle_points_pub_->publish(point_converter_.toRos(output.obstacle_points, header));
  unknown_mask_pub_->publish(point_converter_.toRos(output.unknown_points, header));

  passable_area::msg::TerrainObservability msg;
  msg.header = header;
  msg.frame_partial = output.observability.frame_partial;
  msg.rear_dropout = output.observability.rear_dropout;
  msg.sector_count = static_cast<uint16_t>(output.observability.sectors.size());
  msg.sector_states.reserve(output.observability.sectors.size());
  msg.sector_coverage_confidence.reserve(output.observability.sectors.size());
  for (const auto &sector : output.observability.sectors) {
    msg.sector_states.push_back(static_cast<uint8_t>(sector.state));
    msg.sector_coverage_confidence.push_back(sector.coverage_confidence);
  }
  observability_pub_->publish(msg);
}

} // namespace passable_area::interfaces::ros
