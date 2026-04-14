#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_PERF_STATS_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_PERF_STATS_HPP_

#include "passable_area/interfaces/common/logging/node_logger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace passable_area::interfaces::ros {

class PerfStats {
public:
  void initialize(rclcpp::Node &node,
                  std::shared_ptr<NodeLogger> node_logger = nullptr);
  void observeCloud();
  void observeOdom();
  void record(double processing_ms);

private:
  rclcpp::Node *node_ = nullptr;
  std::shared_ptr<NodeLogger> node_logger_;
  rclcpp::Time window_start_{0, 0, RCL_ROS_TIME};
  int cloud_count_ = 0;
  int odom_count_ = 0;
  int frame_count_ = 0;
  double total_ms_ = 0.0;
  double max_ms_ = 0.0;
};

} // namespace passable_area::interfaces::ros

#endif
