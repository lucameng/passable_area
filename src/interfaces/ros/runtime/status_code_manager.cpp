#include "passable_area/interfaces/ros/runtime/status_code_manager.hpp"

#include "passable_area/interfaces/common/status_code.hpp"

namespace passable_area::interfaces::ros {
namespace {

using passable_area::interfaces::common::StatusCode;
using passable_area::interfaces::common::toInt;
using passable_area::interfaces::common::toString;

} // namespace

void StatusCodeManager::initialize(rclcpp::Node &node,
                                   const RosTopicConfig &topics,
                                   std::shared_ptr<NodeLogger> node_logger,
                                   StatusCodeManagerOptions options) {
  node_ = &node;
  node_logger_ = std::move(node_logger);
  options_ = options;
  current_status_code_ = toInt(StatusCode::OK_RUNNING);
  start_time_ = node.now();
  status_code_pub_ =
      node.create_publisher<std_msgs::msg::Int32>(topics.status_code_topic, 10);
  status_code_timer_ = node.create_wall_timer(
      options_.publish_period, [this]() { statusTimerCallback(); });
}

void StatusCodeManager::markCloud(const rclcpp::Time &stamp) {
  std::lock_guard<std::mutex> lk(mutex_);
  last_cloud_ = stamp;
  seen_cloud_ = true;
}

void StatusCodeManager::markOdom(const rclcpp::Time &stamp) {
  std::lock_guard<std::mutex> lk(mutex_);
  last_odom_ = stamp;
  seen_odom_ = true;
}

void StatusCodeManager::markSynced(const rclcpp::Time &stamp) {
  std::lock_guard<std::mutex> lk(mutex_);
  last_synced_ = stamp;
  seen_synced_ = true;
}

void StatusCodeManager::markOutputSuccess() {
  std::lock_guard<std::mutex> lk(mutex_);
  output_publish_failure_streak_ = 0;
}

void StatusCodeManager::markOutputFailure() {
  std::lock_guard<std::mutex> lk(mutex_);
  ++output_publish_failure_streak_;
}

void StatusCodeManager::markFatalRuntimeException(const std::string &reason) {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (sticky_fatal_status_code_ == 0) {
      sticky_fatal_status_code_ = toInt(StatusCode::FATAL_RUNTIME_EXCEPTION);
    }
  }
  updateStatusCode(toInt(StatusCode::FATAL_RUNTIME_EXCEPTION));
  publishStatusCodeNow(toInt(StatusCode::FATAL_RUNTIME_EXCEPTION));
  if (node_logger_) {
    node_logger_->log(NodeLogger::Level::kError,
                      "Runtime exception promoted status_code to %d(%s): %s",
                      toInt(StatusCode::FATAL_RUNTIME_EXCEPTION),
                      toString(StatusCode::FATAL_RUNTIME_EXCEPTION),
                      reason.c_str());
  } else if (node_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Runtime exception promoted status_code to %d(%s): %s",
                 toInt(StatusCode::FATAL_RUNTIME_EXCEPTION),
                 toString(StatusCode::FATAL_RUNTIME_EXCEPTION), reason.c_str());
  }
}

int32_t StatusCodeManager::currentStatusCode() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return current_status_code_;
}

void StatusCodeManager::statusTimerCallback() {
  if (!node_) {
    return;
  }
  const int32_t status_code = evaluateStatusCode(node_->now());
  updateStatusCode(status_code);
  publishStatusCodeNow(status_code);
}

int32_t StatusCodeManager::evaluateStatusCode(const rclcpp::Time &now) const {
  std::lock_guard<std::mutex> lk(mutex_);
  if (sticky_fatal_status_code_ != 0) {
    return sticky_fatal_status_code_;
  }

  const auto input_timeout =
      rclcpp::Duration::from_seconds(options_.input_timeout_sec);
  const auto sync_timeout =
      rclcpp::Duration::from_seconds(options_.sync_timeout_sec);

  if ((!seen_cloud_ && (now - start_time_) > input_timeout) ||
      (seen_cloud_ && (now - last_cloud_) > input_timeout)) {
    return toInt(StatusCode::ERR_CLOUD_TIMEOUT);
  }
  if ((!seen_odom_ && (now - start_time_) > input_timeout) ||
      (seen_odom_ && (now - last_odom_) > input_timeout)) {
    return toInt(StatusCode::ERR_ODOM_TIMEOUT);
  }
  if (seen_cloud_ && seen_odom_) {
    const rclcpp::Time sync_reference =
        last_cloud_ > last_odom_ ? last_cloud_ : last_odom_;
    if ((!seen_synced_ && (now - sync_reference) > sync_timeout) ||
        (seen_synced_ && (now - last_synced_) > sync_timeout)) {
      return toInt(StatusCode::ERR_SYNC_STALL);
    }
  }
  if (output_publish_failure_streak_ >= options_.output_stall_frames) {
    return toInt(StatusCode::ERR_OUTPUT_STALL);
  }
  return toInt(StatusCode::OK_RUNNING);
}

void StatusCodeManager::publishStatusCodeNow(int32_t status_code) {
  if (!status_code_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = status_code;
  status_code_pub_->publish(msg);
}

void StatusCodeManager::updateStatusCode(int32_t status_code) {
  int32_t previous = 0;
  bool should_log = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (current_status_code_ != status_code) {
      previous = current_status_code_;
      current_status_code_ = status_code;
      if (last_logged_status_code_ != status_code) {
        last_logged_status_code_ = status_code;
        should_log = true;
      }
    }
  }

  if (should_log) {
    logStatusTransition(previous, status_code);
  }
}

void StatusCodeManager::logStatusTransition(int32_t previous, int32_t current) {
  const auto old_code = static_cast<StatusCode>(previous);
  const auto new_code = static_cast<StatusCode>(current);
  if (current >= toInt(StatusCode::FATAL_RUNTIME_EXCEPTION)) {
    if (node_logger_) {
      node_logger_->log(NodeLogger::Level::kError,
                        "status_code changed: %d(%s) -> %d(%s)", previous,
                        toString(old_code), current, toString(new_code));
    } else if (node_) {
      RCLCPP_ERROR(node_->get_logger(), "status_code changed: %d(%s) -> %d(%s)",
                   previous, toString(old_code), current, toString(new_code));
    }
  } else if (current >= toInt(StatusCode::ERR_CLOUD_TIMEOUT)) {
    if (node_logger_) {
      node_logger_->log(NodeLogger::Level::kWarn,
                        "status_code changed: %d(%s) -> %d(%s)", previous,
                        toString(old_code), current, toString(new_code));
    } else if (node_) {
      RCLCPP_WARN(node_->get_logger(), "status_code changed: %d(%s) -> %d(%s)",
                  previous, toString(old_code), current, toString(new_code));
    }
  } else {
    if (node_logger_) {
      node_logger_->log(NodeLogger::Level::kInfo,
                        "status_code changed: %d(%s) -> %d(%s)", previous,
                        toString(old_code), current, toString(new_code));
    } else if (node_) {
      RCLCPP_INFO(node_->get_logger(), "status_code changed: %d(%s) -> %d(%s)",
                  previous, toString(old_code), current, toString(new_code));
    }
  }
}

} // namespace passable_area::interfaces::ros
