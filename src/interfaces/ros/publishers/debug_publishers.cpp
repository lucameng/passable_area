#include "passable_area/interfaces/ros/publishers/debug_publishers.hpp"

namespace passable_area::interfaces::ros {

void DebugPublishers::initialize(
    rclcpp::Node &node, const RosTopicConfig &topics,
    const passable_area::core::DebugConfig &config) {
  publish_base_gravity_cloud_ = config.publish_base_gravity_cloud;
  if (publish_base_gravity_cloud_) {
    base_gravity_cloud_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topics.base_gravity_cloud_topic, 10);
  }
  support_points_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      topics.support_points_topic, 10);
  obstacle_points_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      topics.obstacle_points_topic, 10);
  unknown_mask_pub_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      topics.unknown_mask_topic, 10);
  observability_pub_ =
      node.create_publisher<passable_area::msg::TerrainObservability>(
          topics.observability_topic, 10);
}

void DebugPublishers::publish(const passable_area::core::FrameOutput &output,
                              const std_msgs::msg::Header &header) {
  if (publish_base_gravity_cloud_ && base_gravity_cloud_pub_) {
    base_gravity_cloud_pub_->publish(
        point_converter_.toRos(output.base_gravity_cloud_points, header));
  }
  support_points_pub_->publish(
      point_converter_.toRos(output.support_points, header));
  obstacle_points_pub_->publish(
      point_converter_.toRos(output.obstacle_points, header));
  unknown_mask_pub_->publish(
      point_converter_.toRos(output.unknown_points, header));

  passable_area::msg::TerrainObservability msg;
  msg.header = header;
  msg.frame_partial = output.observability.frame_partial;
  msg.rear_dropout = output.observability.rear_dropout;
  msg.sector_count = static_cast<uint16_t>(output.observability.sectors.size());
  msg.base_point_count = output.base_point_count;
  msg.odom_point_count = output.odom_point_count;
  msg.sector_states.reserve(output.observability.sectors.size());
  msg.sector_coverage_confidence.reserve(output.observability.sectors.size());
  for (const auto &sector : output.observability.sectors) {
    msg.sector_states.push_back(static_cast<uint8_t>(sector.state));
    msg.sector_coverage_confidence.push_back(sector.coverage_confidence);
  }
  observability_pub_->publish(msg);
}

} // namespace passable_area::interfaces::ros
