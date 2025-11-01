#ifndef DR_COMMON_HPP
#define DR_COMMON_HPP

#include <cmath>
#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

// ============================================================================
// Constant Definitions
// ============================================================================

// Mathematical constants
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG2RAD = PI / 180.0f;
constexpr float RAD2DEG = 180.0f / PI;
constexpr float INVALID_VALUE = -100.f;
constexpr float GROUD_HEIGHT = -0.5f;
constexpr int UNKNOWN_OBS_VALID_CNT = 3;

// ============================================================================
// Point Cloud Type Aliases
// ============================================================================

// Basic point cloud types
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = PointCloudXYZ::Ptr;
using PointCloudXYZConstPtr = PointCloudXYZ::ConstPtr;

using PointCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudRGBPtr = PointCloudRGB::Ptr;

using PointCloudI = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudIPtr = PointCloudI::Ptr;

// Point type aliases
using PointXYZ = pcl::PointXYZ;
using PointXYZRGB = pcl::PointXYZRGB;
using PointXYZI = pcl::PointXYZI;

// ============================================================================
// Enumeration Types
// ============================================================================

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

enum class DogModel : uint8_t { X30 = 0, M20 = 1, Unknown = 2 };

// ============================================================================
// Conversion Helpers
// ============================================================================

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

#endif // DR_COMMON_H
