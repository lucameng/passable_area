#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// Basic enum class.
// -----------------------------------------------------------------------------
enum class Passability : uint8_t { Passable = 0, Impassable = 1, Unknown = 2 };

enum class CoverageStatus : uint8_t { Uncovered = 0, Covered = 1 };

enum class Padding : uint8_t { Unpadded = 0, Padded = 1 };

enum class Inpaint : uint8_t {
  MeanOnce,
  Min,
  Max,
  Conditional,
  Mean,
  MinLimit
};

enum class Denoise : uint8_t { Median, Gauss };

enum class CliffState : int8_t { Unknown = -1, NotCliff = 0, Cliff = 1 };

enum class DogModel : uint8_t { X30 = 0, M20 = 1, Unknown = 2 };

// -----------------------------------------------------------------------------
// Basic struct definitions.
// -----------------------------------------------------------------------------

struct BodyGeometry {
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
};

struct PassabilityParams {
  float roughness_threshold = 0.1f;
  float drop_threshold = 0.3f;
  float max_slope_deg = 40.0f;
  bool treat_nan_as_stiff = true;
};

struct LidarParams {
  std::string name;
  Eigen::Vector3f pos_body = Eigen::Vector3f::Zero();     // body frame
  Eigen::Vector3f rpy_body_deg = Eigen::Vector3f::Zero(); // body frame
  Eigen::Vector3f rpy_body_rad = Eigen::Vector3f::Zero(); // body frame
  Eigen::Matrix3f R_mount = Eigen::Matrix3f::Identity();  // rotation

  double fov_up_deg = 52.0;   // deg
  double fov_down_deg = -7.0; // deg
  double fov_up_rad = 0.0;    // deg
  double fov_down_rad = 0.0;  // deg

  double min_range = 0.01; // m
  double max_range = 30.0; // m
};

struct TraversalCostParams {
  bool enabled = false;
  float slope_free_deg = 5.0f;
  float slope_block_deg = 30.0f;
  float rough_free = 0.02f;
  float rough_block = 0.08f;
  float step_free = 0.05f;
  float step_block = 0.18f;
  float slope_weight = 0.4f;
  float roughness_weight = 0.3f;
  float step_weight = 0.3f;
  float easy_cost = 1.0f;
  float hard_cost = 60.0f;
  float max_cost = 100.0f;
  float curve_power = 3.0f;
  int terrain_sample_window = 1;
  float safe_zone_side_length = 1.0f;
};

struct ElevationSolverRegion {
  bool enabled = false;
  float min_x = 0.0f;
  float max_x = 0.0f;
  float min_y = 0.0f;
  float max_y = 0.0f;
};

struct ElevationSolverParams {
  bool use_histogram_solver = true;
  int histogram_bins = 16;
  int min_points = 1;
  int ceiling_window_bins = 2;
  int ceiling_min_points = 1;
  int gap_empty_bins = 2;
  int gap_empty_count_threshold = 0;
  int ground_min_count = 1;
  float float_ratio_threshold = 0.5f;
  int neighbor_min_support = 1;
  float neighbor_height_tolerance = 0.25f;
  ElevationSolverRegion region;
};

struct RaycastParams {
  bool enable = true;
  float max_ray_distance = 4.0f;
  float max_nan_gap = 1.0f;
};

struct ElevationSolverContext {
  int rows = 0;
  int cols = 0;
  int bins = 0;
  float min_height = 0.0f;
  float bin_width = 0.0f;
  const std::vector<uint16_t> *counts = nullptr;
  const std::vector<float> *peaks = nullptr;
  const std::vector<uint8_t> *region_mask = nullptr;
};

struct ElevationSolverResult {
  std::vector<float> ground;
  std::vector<float> ceiling;
  std::vector<float> clearance;
  std::vector<uint8_t> float_mask;
};

// -----------------------------------------------------------------------------
// common used methods.
// -----------------------------------------------------------------------------
inline float toFloat(Passability value) {
  return static_cast<float>(static_cast<uint8_t>(value));
}

inline float toFloat(CoverageStatus value) {
  return static_cast<float>(static_cast<uint8_t>(value));
}

inline float toFloat(Padding value) {
  return static_cast<float>(static_cast<uint8_t>(value));
}

inline Passability toPassability(float value) {
  if (!std::isfinite(value)) {
    return Passability::Unknown;
  }
  const auto code = static_cast<uint8_t>(std::lround(value));
  switch (code) {
  case static_cast<uint8_t>(Passability::Passable):
    return Passability::Passable;
  case static_cast<uint8_t>(Passability::Impassable):
    return Passability::Impassable;
  default:
    return Passability::Unknown;
  }
}

inline CoverageStatus toCoverageStatus(float value) {
  if (!std::isfinite(value)) {
    return CoverageStatus::Uncovered;
  }
  const auto code = static_cast<uint8_t>(std::lround(value));
  switch (code) {
  case static_cast<uint8_t>(CoverageStatus::Covered):
    return CoverageStatus::Covered;
  default:
    return CoverageStatus::Uncovered;
  }
}

#endif // PASSABLE_AREA_COMMON_TYPES_HPP
