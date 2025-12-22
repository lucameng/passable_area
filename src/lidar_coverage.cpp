#include "lidar_coverage.hpp"
#include "common.hpp"
#include "maths.hpp"
#include "utils.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <vector>

LidarCoverage::LidarCoverage(const rclcpp::Node::SharedPtr &node)
    : node_(node), ground_height_(-0.5f), bound_max_({0.5f, 0.25f}),
      bound_min_({-0.5f, -0.25f}), lidar_names_x30_{"lidar_front_up",
                                                    "lidar_front_down",
                                                    "lidar_rear_up",
                                                    "lidar_rear_down"},
      lidar_names_m20_{"lidar_front", "lidar_rear"} {
  // LidarCoverage constructor
}

DogModel LidarCoverage::parseDogModel(const std::string &dog_model) const {
  std::string normalized = dog_model;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (normalized == "x30") {
    return DogModel::X30;
  }
  if (normalized == "m20") {
    return DogModel::M20;
  }
  return DogModel::Unknown;
}

void LidarCoverage::initialize(const std::string &dog_model,
                               const std::vector<LidarParams> &lidar_params) {
  dog_model_ = dog_model;
  lidars_.clear();

  auto normalize = [](std::string model) {
    std::transform(
        model.begin(), model.end(), model.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return model;
  };

  active_model_ = normalize(dog_model_);

  if (lidar_params.empty()) {
    RCLCPP_WARN(
        node_->get_logger(),
        "No lidar_params lidar params provided for model '%s'; coverage "
        "will be empty",
        dog_model_.c_str());
    return;
  }

  lidars_ = lidar_params;
}

void LidarCoverage::processCoverage(grid_map::GridMap &ele_map,
                                    const Eigen::Affine3f &T_g2b) {
  computeCoverage(ele_map, T_g2b, "elevation");
  dilateUncoveredArea(ele_map, 2, "coverability");
}

bool LidarCoverage::isCellCoveredByLidar(const Eigen::Vector3f &p_body,
                                         const LidarParams &lidar) const {
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
