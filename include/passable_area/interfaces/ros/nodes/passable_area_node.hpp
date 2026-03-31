#ifndef PASSABLE_AREA_INTERFACES_ROS_NODES_PASSABLE_AREA_NODE_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_NODES_PASSABLE_AREA_NODE_HPP_

#include "passable_area/interfaces/ros/converters/odom_converter.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"
#include "passable_area/interfaces/ros/publishers/debug_publishers.hpp"
#include "passable_area/interfaces/ros/publishers/result_publishers.hpp"
#include "passable_area/interfaces/ros/status/perf_stats.hpp"
#include "passable_area/interfaces/ros/status/watchdog_manager.hpp"
#include "passable_area/passable_area.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace passable_area::interfaces::ros {

class PassableAreaNode : public rclcpp::Node {
public:
  PassableAreaNode();

private:
  using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
                                                               nav_msgs::msg::Odometry>;

  void onCloudObserved(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void onOdomObserved(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void onSynced(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg);

  passable_area::core::Config config_;
  passable_area::core::Processor processor_;
  RosParamLoader param_loader_;
  PointCloudConverter point_converter_;
  OdomConverter odom_converter_;
  ResultPublishers result_publishers_;
  DebugPublishers debug_publishers_;
  WatchdogManager watchdog_;
  PerfStats perf_stats_;

  std::string input_cloud_topic_;
  std::string odom_topic_;
  int sync_queue_size_ = 10;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

} // namespace passable_area::interfaces::ros

#endif
