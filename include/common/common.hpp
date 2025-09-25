#ifndef COMMON_H
#define COMMON_H

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
constexpr float DEAD_VALUE = -10.f;

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

enum Passability : uint8_t
{
    PASSABLE = 0,
    IMPASSABLE = 1,
    UNKNOWN = 2
};

enum CoverageStatus : uint8_t
{
    UNCOVERED = 0,
    COVERED = 1
};

enum Padding : uint8_t
{
    UNPADDED = 0,
    PADDED = 1,
};

enum Inpaint
{
    MEANONCE,
    MIN,
    MAX,
    CONDITIONAL,
    MEAN,
    MINLIMIT
};

enum Denoise
{
    MEDIAN,
    GAUSS
};

#endif // COMMON_H