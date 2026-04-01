#ifndef PASSABLE_AREA_INTERFACES_ROS_PUBLISHERS_DEBUG_PUBLISHERS_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_PUBLISHERS_DEBUG_PUBLISHERS_HPP_

#include "passable_area/msg/terrain_observability.hpp"
#include "passable_area/core/types/frame_types.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace passable_area::interfaces::ros {

class DebugPublishers {
public:
  void initialize(rclcpp::Node &node, const RosTopicConfig &topics,
                  const passable_area::core::DebugConfig &config);
  void publish(const passable_area::core::FrameOutput &output, const std_msgs::msg::Header &header);

private:
  PointCloudConverter point_converter_;
  bool publish_base_gravity_cloud_ = false;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr base_gravity_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_mask_pub_;
  rclcpp::Publisher<passable_area::msg::TerrainObservability>::SharedPtr observability_pub_;
};

} // namespace passable_area::interfaces::ros

#endif
