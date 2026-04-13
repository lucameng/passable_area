#include "passable_area/interfaces/ros/runtime/perf_stats.hpp"

namespace passable_area::interfaces::ros {

void PerfStats::initialize(rclcpp::Node &node) {
  node_ = &node;
  window_start_ = node.now();
}

void PerfStats::observeCloud() { ++cloud_count_; }

void PerfStats::observeOdom() { ++odom_count_; }

void PerfStats::record(double processing_ms) {
  ++frame_count_;
  total_ms_ += processing_ms;
  max_ms_ = std::max(max_ms_, processing_ms);
  const auto now = node_->now();
  const double window_sec = (now - window_start_).seconds();
  if (window_sec < 1.0) {
    return;
  }
  const double avg_ms =
      frame_count_ > 0 ? total_ms_ / static_cast<double>(frame_count_) : 0.0;
  const double cloud_hz = cloud_count_ / std::max(window_sec, 1e-3);
  const double odom_hz = odom_count_ / std::max(window_sec, 1e-3);
  const double proc_hz = frame_count_ / std::max(window_sec, 1e-3);
  RCLCPP_INFO(node_->get_logger(),
              "perf window: input_hz(cloud/odom)=%.1f/%.1f frames=%d "
              "avg=%.2fms max=%.2fms "
              "rate=%.2fHz",
              cloud_hz, odom_hz, frame_count_, avg_ms, max_ms_, proc_hz);
  window_start_ = now;
  cloud_count_ = 0;
  odom_count_ = 0;
  frame_count_ = 0;
  total_ms_ = 0.0;
  max_ms_ = 0.0;
}

} // namespace passable_area::interfaces::ros
