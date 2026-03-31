#ifndef PASSABLE_AREA_INTERFACES_ROS_STATUS_PERF_STATS_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_STATUS_PERF_STATS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace passable_area::interfaces::ros {

class PerfStats {
public:
  void initialize(rclcpp::Node &node);
  void record(double processing_ms);

private:
  rclcpp::Node *node_ = nullptr;
  rclcpp::Time window_start_{0, 0, RCL_ROS_TIME};
  int frame_count_ = 0;
  double total_ms_ = 0.0;
  double max_ms_ = 0.0;
};

} // namespace passable_area::interfaces::ros

#endif
