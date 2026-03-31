#include "passable_area/interfaces/ros/status/watchdog_manager.hpp"

#include <chrono>

namespace passable_area::interfaces::ros {

void WatchdogManager::initialize(rclcpp::Node &node, const std::string &cloud_topic,
                                 const std::string &odom_topic) {
  node_ = &node;
  cloud_topic_ = cloud_topic;
  odom_topic_ = odom_topic;
  timer_ = node.create_wall_timer(std::chrono::seconds(2), [this]() { check(); });
}

void WatchdogManager::markCloud(const rclcpp::Time &stamp) {
  last_cloud_ = stamp;
  seen_cloud_ = true;
}

void WatchdogManager::markOdom(const rclcpp::Time &stamp) {
  last_odom_ = stamp;
  seen_odom_ = true;
}

void WatchdogManager::markSynced(const rclcpp::Time &stamp) {
  last_synced_ = stamp;
  seen_synced_ = true;
}

void WatchdogManager::check() {
  const auto now = node_->now();
  const auto timeout = rclcpp::Duration::from_seconds(2.0);
  if (!seen_cloud_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Waiting for cloud on '%s'", cloud_topic_.c_str());
  } else if ((now - last_cloud_) > timeout) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "No cloud received for %.2f s", (now - last_cloud_).seconds());
  }
  if (!seen_odom_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Waiting for odom on '%s'", odom_topic_.c_str());
  } else if ((now - last_odom_) > timeout) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "No odom received for %.2f s", (now - last_odom_).seconds());
  }
  if (seen_cloud_ && seen_odom_ && seen_synced_ && (now - last_synced_) > timeout) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "ExactTime has not produced a synced frame for %.2f s",
                         (now - last_synced_).seconds());
  }
}

} // namespace passable_area::interfaces::ros
