#ifndef ELEVATION_MAP_HPP
#define ELEVATION_MAP_HPP

#include "common.hpp"
#include "utils.hpp"

#include <cmath>
#include <deque>
#include <limits>
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
                         float roughness_thres, float drop_thres,
                         const Eigen::Affine3f &T_g2b, bool fill_blind = false);

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

private:
  void setInputCloud(const sensor_msgs::msg::PointCloud2 &ros_cloud);

  bool isIndexValid(const Eigen::Array2i &idx) const noexcept {
    return (idx.x() >= 0 && idx.y() >= 0 && idx.x() < getSize().x() &&
            idx.y() < getSize().y());
  }

  float projectToBodyZ(float scalar_g, const Eigen::Matrix3f &R_g2b) const {
    return dr::projectScalar(scalar_g, R_g2b, Eigen::Vector3f::UnitZ(),
                             Eigen::Vector3f::UnitZ());
  }

  bool isPositionInside(const Eigen::Vector2d &pos) const noexcept {
    return isInside(pos);
  }

  float errorFromCovariance(const Eigen::Vector3f &mean,
                            const Eigen::Matrix3f &square) const;
  float computeError(int x, int y, int kernel_size, Eigen::Vector3f &mean,
                     Eigen::Matrix3f &square) const;
  void inpaint(const std::string &layer_height, const std::string &layer_filled,
               Inpaint method);
  void denoise(const std::string &layer_height, Denoise method,
               int kernel_size);

  void cloud2Elevation();
  bool isPassable(float variance_error, float roughness_thres) const;
  void judgePassability(float rough_thres, float drop_thres, int kernel_size,
                        const Eigen::Affine3f &T_g2b);
  bool isCliffCandidate(int cx, int cy, const Eigen::Affine3f &T_g2b,
                        float drop_thres, int nan_radius_cells,
                        int nan_min_cells, float drop_buffer,
                        float far_distance,
                        std::vector<CliffState> &cliff_cache) const;
  void fillElevationHoles(const std::string &layer_height = "elevation",
                          const std::string &layer_filled = "padding",
                          float search_radius = 1.5f, int min_neighbors = 10,
                          const Eigen::Vector2f &bound_min = {-3.f, -0.7f},
                          const Eigen::Vector2f &bound_max = {4.f, 0.7f});
  void fillPointCloudFromLayer(const std::string &layer_height = "elevation",
                               const std::string &layer_filled = "padding");
  void filterSuspendedObstacles(const std::vector<uint16_t> &histogram,
                                const std::vector<float> &hist_max,
                                float bin_width);
  struct VerticalStructureParams {
    bool use_legacy_vertical = false;
    int bins = 16;
    float ground_quantile = 0.3f;
    int min_points = 1;
    int ceiling_window_bins = 2;
    int ceiling_min_points = 6;
    int gap_empty_bins = 2;
    int gap_empty_count_threshold = 0;
    float float_ratio_threshold = 0.8f;
    int neighbor_min_support = 0;
    float neighbor_height_tolerance = 0.25f;
  };
  void analyzeVerticalStructure(const std::vector<uint16_t> &histogram,
                                const std::vector<float> &hist_max,
                                float bin_width);
  int findGroundBin(const uint16_t *cell_hist, int bins, int total_points,
                    bool &gap_found) const;
  int findCeilingBin(const uint16_t *cell_hist, int bins, int ground_bin,
                     bool gap_found) const;
  int computeMaxGap(const uint16_t *cell_hist, int ground_bin,
                    int ceiling_bin) const;
  float computeClusterRatio(const uint16_t *cell_hist, int bins,
                            int ceiling_bin, int total_points) const;
  int countNeighborSupport(const std::vector<float> &ceiling_buffer,
                           const std::vector<bool> &ceiling_found, int rows,
                           int cols, int r, int c, float ceiling_z) const;
  float binTop(int bin, float bin_width) const noexcept;
  float binCenter(int bin, float bin_width) const noexcept;
  float binBottom(int bin, float bin_width) const noexcept;
  float binMaxHeight(const std::vector<float> &hist_max, int linear, int bins,
                     int bin) const noexcept;
  int linearIndex(int r, int c) const noexcept;

  void setUseLegacyVertical(bool enable) noexcept {
    vs_params_.use_legacy_vertical = enable;
  }

private:
  PointCloudXYZ working_cloud_;
  float map_length_;
  float map_width_;
  float min_height_;
  float max_height_;
  float grid_size_;
  grid_map::Size map_cells_;
  std::string frame_;
  rclcpp::Logger logger_;
  VerticalStructureParams vs_params_;
};

#endif // ELEVATION_MAP_HPP
