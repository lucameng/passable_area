#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_WATCHDOG_MANAGER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_WATCHDOG_MANAGER_HPP_

#include "passable_area/interfaces/common/logging/node_logger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace passable_area::interfaces::ros {

class WatchdogManager {
public:
  void initialize(rclcpp::Node &node, const std::string &cloud_topic,
                  const std::string &odom_topic,
                  std::shared_ptr<NodeLogger> node_logger = nullptr);
  void markCloud(const rclcpp::Time &stamp);
  void markOdom(const rclcpp::Time &stamp);
  void markSynced(const rclcpp::Time &stamp);

private:
  void check();
  void logWaitingForCloud();
  void logWaitingForOdom();

  rclcpp::Node *node_ = nullptr;
  std::shared_ptr<NodeLogger> node_logger_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_cloud_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_synced_{0, 0, RCL_ROS_TIME};
  bool seen_cloud_ = false;
  bool seen_odom_ = false;
  bool seen_synced_ = false;
  bool logged_waiting_cloud_ = false;
  bool logged_waiting_odom_ = false;
  std::string cloud_topic_;
  std::string odom_topic_;
};

} // namespace passable_area::interfaces::ros

#endif
