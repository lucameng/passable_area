#ifndef ELEVATION_MAP_HPP
#define ELEVATION_MAP_HPP

#include "elevation_solver.hpp"
#include "common_types.hpp"
#include "common.hpp"
#include "utils.hpp"

#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ElevationMap : public grid_map::GridMap {
public:
  ElevationMap() = default;
  explicit ElevationMap(
      float map_length, float map_width, float min_height, float max_height,
      float grid_s, const std::string &frame_id,
      const rclcpp::Logger &logger = rclcpp::get_logger("ElevationMap"));

  void processPointCloud(const sensor_msgs::msg::PointCloud2 &ros_cloud,
                         const PassabilityParams &pass_params,
                         const Eigen::Affine3f &T_g2b);

  const PointCloudXYZ &getWorkingCloud() const noexcept {
    return working_cloud_;
  }
  PointCloudXYZ &getWorkingCloud() noexcept { return working_cloud_; }

  float getMinheight() const noexcept;
  float getMaxheight() const noexcept;

  void setAltitude(const Eigen::Array2i &idx, float height);
  void setAltitude(const Eigen::Vector2d &pos, float height);
  void setGroundHeight(const Eigen::Array2i &idx, float height);
  void setGroundHeight(const Eigen::Vector2d &pos, float height);

  float getAltitude(const Eigen::Array2i &idx) const;
  float getAltitude(const Eigen::Vector2d &pos) const;
  float getGroundHeight(const Eigen::Array2i &idx) const;
  float getGroundHeight(const Eigen::Vector2d &pos) const;

  void setPassability(const Eigen::Array2i &idx, Passability passability);
  void setPassability(const Eigen::Vector2d &pos, Passability passability);

  Passability getPassability(const Eigen::Array2i &idx) const;
  Passability getPassability(const Eigen::Vector2d &pos) const;

  CoverageStatus getCoverability(const Eigen::Array2i &idx) const;
  CoverageStatus getCoverability(const Eigen::Vector2d &pos) const;

  int getPointCount(const Eigen::Array2i &idx) const;
  int getPointCount(const Eigen::Vector2d &pos) const;

  float getGridSize() const noexcept { return grid_size_; }
  grid_map::Size getCellSize() const noexcept { return map_cells_; }
  float getBaselineGround(float radius) const;
  void setMaxInpaintPixels(int max_pixels) noexcept;
  void setCenterPaddingParams(bool enabled, float radius) noexcept;
  void setElevationSolverParams(const ElevationSolverParams &params) noexcept;
  void setTraversalCostParams(const TraversalCostParams &params) noexcept;
  void setRaycastParams(const RaycastParams &params) noexcept;
  void setLidarParams(const std::vector<LidarParams> &lidars) noexcept {
    raycast_lidars_ = lidars;
  }
  void setCurrentTransform(const Eigen::Affine3f &T_g2b) noexcept {
    current_T_g2b_ = T_g2b;
  }
  const Eigen::Affine3f &getCurrentTransform() const noexcept {
    return current_T_g2b_;
  }
  float projectToBodyZ(float scalar_g, const Eigen::Matrix3f &R_g2b) const {
    return dr::projectScalar(scalar_g, R_g2b, Eigen::Vector3f::UnitZ(),
                             Eigen::Vector3f::UnitZ());
  }
  const TraversalCostParams &getTraversalCostParams() const noexcept {
    return traversal_params_;
  }
  float computeRoughness(int x, int y, int kernel_size, Eigen::Vector3f &mean,
                         Eigen::Matrix3f &square) const;
  float computeSlopeRad(int row, int col,
                        const grid_map::Matrix &elevation) const;
  float computeStepHeight(int row, int col,
                          const grid_map::Matrix &elevation) const;
  float normalizeMetric(float value, float free_threshold,
                        float block_threshold) const noexcept;

private:
  void setInputCloud(const sensor_msgs::msg::PointCloud2 &ros_cloud);

  bool isIndexValid(const Eigen::Array2i &idx) const noexcept {
    return (idx.x() >= 0 && idx.y() >= 0 && idx.x() < getSize().x() &&
            idx.y() < getSize().y());
  }

  bool isPositionInside(const Eigen::Vector2d &pos) const noexcept {
    return isInside(pos);
  }

  float extractVariance(const Eigen::Vector3f &mean,
                        const Eigen::Matrix3f &square) const;
  void inpaint(const std::string &layer_height, const std::string &layer_filled,
               Inpaint method);
  void denoise(const std::string &layer_height, Denoise method,
               int kernel_size);

  void cloud2Elevation();
  bool isPassable(float roughness_value, float roughness_thres) const;
  void judgePassability(float rough_thres, float drop_thres,
                        float max_slope_deg, int kernel_size,
                        bool treat_nan_as_stiff);
  std::optional<std::reference_wrapper<const LidarParams>>
  selectRaycastLidar(const Eigen::Vector3f &target_body) const;
  bool hasCliffDropOnRay(const grid_map::Index &target_idx,
                         const grid_map::Matrix &elevation,
                         float drop_thres) const;
  bool isCliffCandidate(int cx, int cy, const Eigen::Affine3f &T_g2b,
                        float drop_thres, int nan_radius_cells,
                        int nan_min_cells, float drop_buffer,
                        float far_distance,
                        std::vector<CliffState> &cliff_cache) const;
private:
  PointCloudXYZ working_cloud_;
  float map_length_;
  float map_width_;
  float min_height_;
  float max_height_;
  float grid_size_;
  int max_inpaint_pixels_;
  bool center_padding_enabled_;
  float center_padding_radius_;
  grid_map::Size map_cells_;
  Eigen::Affine3f current_T_g2b_;
  std::string frame_;
  rclcpp::Logger logger_;
  ElevationSolverParams solver_params_;
  TraversalCostParams traversal_params_;
  RaycastParams raycast_params_;
  std::vector<LidarParams> raycast_lidars_;
};

#endif // ELEVATION_MAP_HPP
