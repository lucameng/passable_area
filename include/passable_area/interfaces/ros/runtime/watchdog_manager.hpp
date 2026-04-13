#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_WATCHDOG_MANAGER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_WATCHDOG_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

namespace passable_area::interfaces::ros {

class WatchdogManager {
public:
  void initialize(rclcpp::Node &node, const std::string &cloud_topic,
                  const std::string &odom_topic);
  void markCloud(const rclcpp::Time &stamp);
  void markOdom(const rclcpp::Time &stamp);
  void markSynced(const rclcpp::Time &stamp);

private:
  void check();

  rclcpp::Node *node_ = nullptr;
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
