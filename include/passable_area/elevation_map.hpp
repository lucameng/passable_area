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
#include <sensor_msgs/msg/point_cloud2.hpp>

class ElevationMap : public grid_map::GridMap {
public:
  ElevationMap() = default;
  explicit ElevationMap(float map_length, float map_width, float min_height,
                        float max_height, float grid_s,
                        const std::string &frame_id);

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

  float minCoeffOfFinites(const Eigen::MatrixXf &mat) const noexcept {
    return mat.array()
        .isFinite()
        .select(mat, std::numeric_limits<float>::max())
        .minCoeff();
  }

  float maxCoeffOfFinites(const Eigen::MatrixXf &mat) const noexcept {
    return mat.array()
        .isFinite()
        .select(mat, std::numeric_limits<float>::lowest())
        .maxCoeff();
  }

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

  void minValues(const std::string &layer_height,
                 const std::string &layer_filled);
  void minValuesLimited(const std::string &layer_height,
                        const std::string &layer_filled,
                        int max_hole_pixels = 10,
                        bool enable_center_padding = false,
                        float center_dist_thresh = 0.8f);
  void maxValues(const std::string &layer_height,
                 const std::string &layer_filled);
  void meanValues(const std::string &layer_height,
                  const std::string &layer_filled);
  void meanValuesOnce(const std::string &layer_height,
                      const std::string &layer_filled);

  void medianFilter(const std::string &layer_height, int kernel_size,
                    float threshold = -std::numeric_limits<float>::infinity());
  void gaussianFilter(const std::string &layer_height, int kernel_size);

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
                        std::vector<int8_t> &cliff_cache) const;
  void fillElevationHoles(const std::string &layer_height = "elevation",
                          const std::string &layer_filled = "padding",
                          float search_radius = 1.5f, int min_neighbors = 10,
                          const Eigen::Vector2f &bound_min = {-3.f, -0.7f},
                          const Eigen::Vector2f &bound_max = {4.f, 0.7f});
  void fillPointCloudFromLayer(const std::string &layer_height = "elevation",
                               const std::string &layer_filled = "padding");

private:
  PointCloudXYZ working_cloud_;
  float map_length_;
  float map_width_;
  float min_height_;
  float max_height_;
  float grid_size_;
  grid_map::Size map_cells_;
  std::string frame_;
  struct Pixel {
    int r, c;
  };
};

#endif // ELEVATION_MAP_HPP
