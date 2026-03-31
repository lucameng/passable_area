#include "passable_area/interfaces/ros/status/perf_stats.hpp"

namespace passable_area::interfaces::ros {

void PerfStats::initialize(rclcpp::Node &node) {
  node_ = &node;
  window_start_ = node.now();
}

void PerfStats::record(double processing_ms) {
  ++frame_count_;
  total_ms_ += processing_ms;
  max_ms_ = std::max(max_ms_, processing_ms);
  const auto now = node_->now();
  if ((now - window_start_).seconds() < 5.0) {
    return;
  }
  const double avg_ms = frame_count_ > 0 ? total_ms_ / static_cast<double>(frame_count_) : 0.0;
  RCLCPP_INFO(node_->get_logger(),
              "perf window: frames=%d avg=%.2fms max=%.2fms rate=%.2fHz", frame_count_, avg_ms,
              max_ms_, frame_count_ / std::max((now - window_start_).seconds(), 1e-3));
  window_start_ = now;
  frame_count_ = 0;
  total_ms_ = 0.0;
  max_ms_ = 0.0;
}

} // namespace passable_area::interfaces::ros
