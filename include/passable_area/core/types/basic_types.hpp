#ifndef PASSABLE_AREA_CORE_TYPES_BASIC_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_BASIC_TYPES_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

namespace passable_area::core {

using Timestamp = int64_t;

struct Point3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

using PointCloud = std::vector<Point3f>;

struct Pose3D {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
};

} // namespace passable_area::core

#endif
