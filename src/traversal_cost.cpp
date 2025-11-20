#include "traversal_cost.hpp"
#include "elevation_map.hpp"
#include "maths.hpp"
#include "utils.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <exception>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>

TraversalCost::TraversalCost(ElevationMap &map,
                             const TraversalCostParams &params,
                             const rclcpp::Logger &logger)
    : map_(map), params_(params), logger_(logger) {}

bool TraversalCost::updateCostLayer() {
  if (!map_.exists("traversal_cost") || !map_.exists("elevation"))
    return false;

  auto &slope_layer = map_.get("slope");
  auto &rough_layer = map_.get("roughness");
  auto &step_layer = map_.get("step_height");
  auto &cost_layer = map_.get("traversal_cost");
  const auto &elevation_layer = map_.get("elevation");

  slope_layer.setConstant(std::numeric_limits<float>::quiet_NaN());

  const auto params = map_.getTraversalCostParams();
  if (!params.enabled) {
    cost_layer.setConstant(std::numeric_limits<float>::quiet_NaN());
    return false;
  }

  cost_layer.setConstant(params.easy_cost);

  const auto size = map_.getSize();
  const int rows = size.x();
  const int cols = size.y();
  const float weight_sum =
      std::max(1e-3f, params.slope_weight + params.roughness_weight +
                          params.step_weight);
  const int rough_kernel =
      std::clamp(2 * params.terrain_sample_window + 1, 3, 7);
  const float slope_free_rad = static_cast<float>(
      dr::degreeToRadian(static_cast<double>(params.slope_free_deg)));
  const float slope_block_rad = static_cast<float>(
      dr::degreeToRadian(static_cast<double>(params.slope_block_deg)));
  const Eigen::Matrix3f R_g2b = map_.getCurrentTransform().linear();
  const float half_extent = 0.5f * params.safe_zone_side_length;

  auto fallback_roughness = [&](int row, int col) -> float {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    Eigen::Matrix3f square = Eigen::Matrix3f::Zero();
    float variance =
        map_.computeRoughness(row, col, rough_kernel, mean, square);
    if (variance <= 0.0f)
      return std::numeric_limits<float>::quiet_NaN();
    return std::sqrt(std::max(0.0f, variance));
  };

  auto fallback_step = [&](int row, int col) -> float {
    const float center = elevation_layer(row, col);
    if (!std::isfinite(center))
      return std::numeric_limits<float>::quiet_NaN();
    float max_diff = 0.0f;
    bool has_neighbor = false;
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0)
          continue;
        const int rr = row + dr;
        const int cc = col + dc;
        if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
          continue;
        const float nh = elevation_layer(rr, cc);
        if (!std::isfinite(nh))
          continue;
        has_neighbor = true;
        const float diff = std::fabs(map_.projectToBodyZ(center - nh, R_g2b));
        if (diff > max_diff)
          max_diff = diff;
      }
    }
    if (!has_neighbor)
      return std::numeric_limits<float>::quiet_NaN();
    return max_diff;
  };

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const float height = elevation_layer(r, c);
      if (!std::isfinite(height)) {
        cost_layer(r, c) = params.easy_cost;
        continue;
      }

      const float slope_rad = map_.computeSlopeRad(r, c, elevation_layer);
      float roughness = rough_layer(r, c);
      float step = step_layer(r, c);
      if (!std::isfinite(roughness)) {
        roughness = fallback_roughness(r, c);
        if (std::isfinite(roughness)) {
          rough_layer(r, c) = roughness;
        }
      }
      if (!std::isfinite(step)) {
        step = fallback_step(r, c);
        if (std::isfinite(step)) {
          step_layer(r, c) = step;
        }
      }

      grid_map::Position pos;
      const bool have_pos = map_.getPosition(grid_map::Index(r, c), pos);
      const bool in_safe_zone = have_pos && std::fabs(pos.x()) <= half_extent &&
                                std::fabs(pos.y()) <= half_extent;

      if (!std::isfinite(roughness) || !std::isfinite(step)) {
        cost_layer(r, c) = params.easy_cost;
        continue;
      }

      slope_layer(r, c) = slope_rad;

      if (in_safe_zone) {
        cost_layer(r, c) = params.easy_cost;
        continue;
      }

      const bool beyond_limit = slope_rad >= slope_block_rad ||
                                roughness >= params.rough_block ||
                                step >= params.step_block;
      if (beyond_limit) {
        cost_layer(r, c) = params.max_cost;
        continue;
      }

      const float slope_ratio =
          map_.normalizeMetric(slope_rad, slope_free_rad, slope_block_rad);
      const float rough_ratio = map_.normalizeMetric(
          roughness, params.rough_free, params.rough_block);
      const float step_ratio =
          map_.normalizeMetric(step, params.step_free, params.step_block);

      float difficulty = (slope_ratio * params.slope_weight +
                          rough_ratio * params.roughness_weight +
                          step_ratio * params.step_weight) /
                         weight_sum;
      difficulty = std::clamp(difficulty, 0.0f, 1.0f);

      const float curve_power = std::max(0.1f, params.curve_power);
      const float shaped = std::pow(difficulty, curve_power);
      float cost =
          params.easy_cost + (params.hard_cost - params.easy_cost) * shaped;
      cost = std::clamp(cost, params.easy_cost, params.hard_cost);
      cost_layer(r, c) = cost;
    }
  }

  return true;
}

bool TraversalCost::publish(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher,
    const std::string &frame_id, const rclcpp::Time &stamp) const {
  if (!publisher)
    return false;

  nav_msgs::msg::OccupancyGrid msg;
  if (!fillMessage(msg, frame_id, stamp))
    return false;

  publisher->publish(msg);
  return true;
}

bool TraversalCost::fillMessage(nav_msgs::msg::OccupancyGrid &msg,
                                const std::string &frame_id,
                                const rclcpp::Time &stamp) const {
  const auto params = map_.getTraversalCostParams();
  if (!params.enabled || !map_.exists("traversal_cost"))
    return false;

  try {
    grid_map::GridMapRosConverter::toOccupancyGrid(
        map_, "traversal_cost", params.easy_cost, params.max_cost, msg);
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "Failed to convert traversal cost layer: %s",
                e.what());
    return false;
  }
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  return true;
}
