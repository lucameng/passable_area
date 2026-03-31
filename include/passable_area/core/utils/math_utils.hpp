#ifndef PASSABLE_AREA_CORE_UTILS_MATH_UTILS_HPP_
#define PASSABLE_AREA_CORE_UTILS_MATH_UTILS_HPP_

#include <Eigen/Geometry>

#include <cmath>

namespace passable_area::core {

inline float YawFromQuaternion(const Eigen::Quaternionf &orientation) {
  const Eigen::Quaternionf normalized = orientation.normalized();
  const float x = normalized.x();
  const float y = normalized.y();
  const float z = normalized.z();
  const float w = normalized.w();
  return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

} // namespace passable_area::core

#endif
