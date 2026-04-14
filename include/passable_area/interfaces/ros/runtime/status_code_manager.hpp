#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_STATUS_CODE_MANAGER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_STATUS_CODE_MANAGER_HPP_

#include "passable_area/interfaces/common/logging/node_logger.hpp"
#include "passable_area/interfaces/ros/ros_param_loader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace passable_area::interfaces::ros {

struct StatusCodeManagerOptions {
  double input_timeout_sec = 2.0;
  double sync_timeout_sec = 2.0;
  int output_stall_frames = 5;
  std::chrono::milliseconds publish_period{100};
};

class StatusCodeManager {
public:
  void initialize(rclcpp::Node &node, const RosTopicConfig &topics,
                  std::shared_ptr<NodeLogger> node_logger = nullptr,
                  StatusCodeManagerOptions options = {});
  void markCloud(const rclcpp::Time &stamp);
  void markOdom(const rclcpp::Time &stamp);
  void markSynced(const rclcpp::Time &stamp);
  void markOutputSuccess();
  void markOutputFailure();
  void markFatalRuntimeException(const std::string &reason);
  int32_t currentStatusCode() const;

private:
  void statusTimerCallback();
  int32_t evaluateStatusCode(const rclcpp::Time &now) const;
  void publishStatusCodeNow(int32_t status_code);
  void updateStatusCode(int32_t status_code);
  void logStatusTransition(int32_t previous, int32_t current);

  rclcpp::Node *node_ = nullptr;
  std::shared_ptr<NodeLogger> node_logger_;
  StatusCodeManagerOptions options_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_code_pub_;
  rclcpp::TimerBase::SharedPtr status_code_timer_;

  mutable std::mutex mutex_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cloud_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_synced_{0, 0, RCL_ROS_TIME};
  bool seen_cloud_ = false;
  bool seen_odom_ = false;
  bool seen_synced_ = false;
  int output_publish_failure_streak_ = 0;
  int32_t current_status_code_ = 100;
  int32_t sticky_fatal_status_code_ = 0;
  int32_t last_logged_status_code_ = -1;
};

} // namespace passable_area::interfaces::ros

#endif
