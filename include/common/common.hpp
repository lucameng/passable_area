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

#endif // DR_COMMON_H
