#include "passable_area/interfaces/ros/converters/odom_converter.hpp"

#include <cmath>

namespace passable_area::interfaces::ros {

bool OdomConverter::fromRos(const nav_msgs::msg::Odometry &msg,
                            passable_area::core::Pose3D &pose) const {
  const auto &p = msg.pose.pose.position;
  const auto &q = msg.pose.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      !std::isfinite(q.w)) {
    return false;
  }
  pose.position = Eigen::Vector3f(p.x, p.y, p.z);
  pose.orientation = Eigen::Quaternionf(q.w, q.x, q.y, q.z);
  if (pose.orientation.norm() < 1e-4f) {
    return false;
  }
  pose.orientation.normalize();
  return true;
}

} // namespace passable_area::interfaces::ros
