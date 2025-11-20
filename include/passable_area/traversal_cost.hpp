#ifndef PASSABLE_AREA_TRAVERSAL_COST_HPP
#define PASSABLE_AREA_TRAVERSAL_COST_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

struct TraversalCostParams {
  bool enabled{false};
  float slope_free_deg{5.0f};
  float slope_block_deg{30.0f};
  float rough_free{0.02f};
  float rough_block{0.08f};
  float step_free{0.05f};
  float step_block{0.18f};
  float slope_weight{0.4f};
  float roughness_weight{0.3f};
  float step_weight{0.3f};
  float easy_cost{1.0f};
  float hard_cost{60.0f};
  float max_cost{100.0f};
  float curve_power{3.0f};
  int terrain_sample_window{1};
  float safe_zone_side_length{1.0f};
};

class ElevationMap;

class TraversalCost {
public:
  TraversalCost(
      ElevationMap &map, const TraversalCostParams &params,
      const rclcpp::Logger &logger = rclcpp::get_logger("TraversalCost"));

  void setParams(const TraversalCostParams &params) noexcept;
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

#endif // PASSABLE_AREA_TRAVERSAL_COST_HPP
