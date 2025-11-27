#ifndef TRAVERSAL_COST_HPP
#define TRAVERSAL_COST_HPP

#include "common_types.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

class ElevationMap;

class TraversalCost {
public:
  TraversalCost(
      ElevationMap &map, const TraversalCostParams &params,
      const rclcpp::Logger &logger = rclcpp::get_logger("TraversalCost"));

  const TraversalCostParams &getParams() const noexcept { return params_; }

  bool updateCostLayer();
  bool publish(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
                   &publisher,
               const std::string &frame_id, const rclcpp::Time &stamp) const;

private:
  bool fillMessage(nav_msgs::msg::OccupancyGrid &msg,
                   const std::string &frame_id,
                   const rclcpp::Time &stamp) const;

  ElevationMap &map_;
  TraversalCostParams params_;
  rclcpp::Logger logger_;
};

#endif // TRAVERSAL_COST_HPP
