#ifndef PASSABLE_AREA_INTERFACES_ROS_PASSABLE_AREA_NODE_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_PASSABLE_AREA_NODE_HPP_

#include "passable_area/interfaces/common/logging/log_bridge.hpp"
#include "passable_area/interfaces/common/logging/node_logger.hpp"
#include "passable_area/interfaces/ros/converters/odom_converter.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/interfaces/ros/ros_param_loader.hpp"
#include "passable_area/interfaces/ros/runtime/debug_publishers.hpp"
#include "passable_area/interfaces/ros/runtime/perf_stats.hpp"
#include "passable_area/interfaces/ros/runtime/result_publishers.hpp"
#include "passable_area/interfaces/ros/runtime/status_code_manager.hpp"
#include "passable_area/interfaces/ros/runtime/watchdog_manager.hpp"
#include "passable_area/passable_area.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cstdint>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace passable_area::interfaces::ros {

class PassableAreaNode : public rclcpp::Node {
public:
  explicit PassableAreaNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  using SyncPolicy =
      message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
                                                nav_msgs::msg::Odometry>;

  void
  onCloudObserved(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void onOdomObserved(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void onSynced(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);
  bool initDrLogger();
  void logDebug(const char *format, ...);
  void logInfo(const char *format, ...);
  void logWarn(const char *format, ...);
  void logWarnThrottle(uint64_t throttle_ms, const char *key,
                       const char *format, ...);
  void logError(const char *format, ...);
  void logFatal(const char *format, ...);
  void publishBaseGravityTransform(
      const passable_area::core::Pose3D &base_pose_in_odom,
      const builtin_interfaces::msg::Time &stamp);

  RosNodeParams node_params_;
  passable_area::core::Config config_;
  passable_area::core::Processor processor_;
  RosTopicConfig topic_config_;
  PointCloudConverter point_converter_;
  OdomConverter odom_converter_;
  ResultPublishers result_publishers_;
  DebugPublishers debug_publishers_;
  StatusCodeManager status_code_manager_;
  WatchdogManager watchdog_;
  PerfStats perf_stats_;
  std::shared_ptr<LogBridge> log_bridge_;
  std::shared_ptr<NodeLogger> node_logger_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} // namespace passable_area::interfaces::ros

#endif
