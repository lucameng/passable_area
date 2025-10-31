#include "lidar_coverage.hpp"
#include "common.hpp"
#include "maths.hpp"
#include "utils.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

LidarCoverage::LidarCoverage(const rclcpp::Node::SharedPtr &node)
    : node_(node), ground_height_(-0.5f), bound_max_({4.f, 0.7f}),
      bound_min_({-3.f, -0.7f}) {
  // LidarCoverage constructor
}

void LidarCoverage::initialize() {
  // imu_sub_ = nh_.subscribe("imu", 1000, &LidarCoverage::imuCallback, this);
  addLidar("lidar_front_up");
  addLidar("lidar_front_down");
  addLidar("lidar_rear_up");
  addLidar("lidar_rear_down");
}

void LidarCoverage::addLidar(const std::string &lidar_name) {
  LidarParam param;
  param.name = lidar_name;

  // ----------- pos_body -----------
  std::vector<double> pos_vec;
  node_->declare_parameter(lidar_name + ".pos_body",
                           std::vector<double>{0.0, 0.0, 0.0});
  node_->get_parameter(lidar_name + ".pos_body", pos_vec);

  if (pos_vec.size() == 3) {
    param.pos_body = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get position for %s",
                 lidar_name.c_str());
  }

  // ----------- rpy_body -----------
  std::vector<double> rpy_vec;
  node_->declare_parameter(lidar_name + ".rpy_body",
                           std::vector<double>{0.0, 0.0, 0.0});
  node_->get_parameter(lidar_name + ".rpy_body", rpy_vec);

  if (rpy_vec.size() == 3) {
    param.rpy_body_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
    param.rpy_body_rad = Eigen::Vector3f(dr::degreeToRadian(rpy_vec[0]),
                                         dr::degreeToRadian(rpy_vec[1]),
                                         dr::degreeToRadian(rpy_vec[2]));
    param.R_mount = dr::rotationFromYPRrad(param.rpy_body_rad);

    RCLCPP_INFO(node_->get_logger(), "%s:\nrpy(deg): [%.2f %.2f %.2f]",
                lidar_name.c_str(), rpy_vec[0], rpy_vec[1], rpy_vec[2]);
    RCLCPP_INFO(node_->get_logger(),
                "%s:\nrpy(deg): [%.2f %.2f %.2f]\nrotation matrix:\n%s",
                lidar_name.c_str(), rpy_vec[0], rpy_vec[1], rpy_vec[2],
                dr::toString(param.R_mount).c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get euler angle for %s",
                 lidar_name.c_str());
  }

  // ----------- fov_up / fov_down -----------
  node_->declare_parameter(lidar_name + ".fov_up_deg", 15.0);
  node_->declare_parameter(lidar_name + ".fov_down_deg", -15.0);

  node_->get_parameter(lidar_name + ".fov_up_deg", param.fov_up_deg);
  node_->get_parameter(lidar_name + ".fov_down_deg", param.fov_down_deg);

  param.fov_up_rad = dr::degreeToRadian(param.fov_up_deg);
  param.fov_down_rad = dr::degreeToRadian(param.fov_down_deg);

  lidars_.push_back(param);
}

void LidarCoverage::processCoverage(grid_map::GridMap &ele_map,
                                    const Eigen::Affine3f &T_g2b) {
  computeCoverage(ele_map, T_g2b, "dummy_height");
  dilateUncoveredArea(ele_map, 2, "coverability");
}

bool LidarCoverage::isCellCoveredByLidar(const Eigen::Vector3f &p_body,
                                         const LidarParam &lidar) const {
  Eigen::Vector3f v_body = p_body - lidar.pos_body;
  float dist = v_body.norm();
  if (dist < lidar.min_range || dist > lidar.max_range)
    return false;

  Eigen::Vector3f v_lidar = lidar.R_mount.transpose() * v_body;
  // Eigen::Vector3f v_lidar = lidar.R_mount * v_body;

  // if (v_lidar.x() <= 0.0) return false;
  // ROS_INFO_STREAM("\nv_body:\n" << v_body << "\nv_lidar:\n"<< v_lidar <<
  // "\n");
  float horiz = std::hypot(v_lidar.x(), v_lidar.y());
  float theta = std::atan2(v_lidar.z(), horiz); // rad

  // ROS_INFO("horiz=%.3f theta=%.3f", horiz, dr::radianToDegree(theta));
  if (theta >= lidar.fov_down_rad && theta <= lidar.fov_up_rad)
    return true;
  return false;
}

void LidarCoverage::computeCoverage(grid_map::GridMap &ele_map,
                                    const Eigen::Affine3f &T_g2b,
                                    const std::string &layer_height,
                                    const std::string &layer_covered) const {
  // ROS_INFO("Start computing lidar coverage...");
  if (!ele_map.exists(layer_covered) || !ele_map.exists(layer_height))
    return;

  ele_map.get(layer_covered).setConstant(toFloat(CoverageStatus::Covered));

  for (grid_map::GridMapIterator it(ele_map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx = *it;
    grid_map::Position3 pos;
    if (!ele_map.getPosition3(layer_height, idx, pos))
      continue;

    if (pos.x() < bound_min_.x() || pos.x() > bound_max_.x() ||
        pos.y() < bound_min_.y() || pos.y() > bound_max_.y()) {
      continue;
    }

    Eigen::Vector3f p_grav = {static_cast<float>(pos.x()),
                              static_cast<float>(pos.y()),
                              static_cast<float>(pos.z())};
    Eigen::Vector3f p_body = T_g2b * p_grav;

    // ROS_INFO("p_grav:(%.3f, %.3f, %.3f) p_body(%.3f, %.3f, %.3f)",
    //         p_grav.x(), p_grav.y(), p_grav.z(), p_body.x(), p_body.y(),
    //         p_body.z());

    bool is_covered = false;
    for (const auto &lidar : lidars_) {
      if (isCellCoveredByLidar(p_body, lidar)) {
        // ele_map.at(layer_covered, idx) =
        // toFloat(CoverageStatus::Covered);
        is_covered = true;
        break;
      }
    }

    if (!is_covered) {
      ele_map.at(layer_covered, idx) = toFloat(CoverageStatus::Uncovered);
    }
  }
  // ROS_INFO("Computation done!");
}

void LidarCoverage::dilateUncoveredArea(grid_map::GridMap &ele_map,
                                        int dilation_radius,
                                        const std::string &layer_covered) {
  if (!ele_map.exists(layer_covered)) {
    return;
  }

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
      ele_map, layer_covered, CV_8UC1, 0.0, 1.0, image);

  cv::Mat image_eroded;
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(2 * dilation_radius + 1, 2 * dilation_radius + 1));
  cv::erode(image, image_eroded, kernel);

  cv::Mat image_float;
  image_eroded.convertTo(image_float, CV_32F, 1.0 / 255.0);
  cv::cv2eigen(image_float, ele_map[layer_covered]);
}
