#include "passable_area_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>

PassableAreaNode::PassableAreaNode(const ros::NodeHandle &nh,
                                   const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), stamp_(0), ele_init_(false), lidar_init_(false),
      enable_blind_check_(false), map_length_(10.0f), map_width_(10.0f),
      min_height_(-1.5f), max_height_(1.5f), voxel_width_(0.1f),
      max_inpaint_pixels_(200), enable_center_padding_(true),
      center_dist_thresh_(0.8f), body_length_(0.0f), body_width_(0.0f),
      body_height_(0.0f), T_g2b_(Eigen::Affine3f::Identity()) {
  pnh_.param("enable_blind_check", enable_blind_check_, false);
  pnh_.param("map_length", map_length_, 10.0f);
  pnh_.param("map_width", map_width_, 10.0f);
  pnh_.param("map_height_min", min_height_, -1.5f);
  pnh_.param("map_height_max", max_height_, 1.5f);
  pnh_.param("voxel_size", voxel_width_, 0.1f);
  pnh_.param("max_inpaint_pixels", max_inpaint_pixels_, 200);
  pnh_.param("enable_center_padding", enable_center_padding_, true);
  pnh_.param("center_dist_thresh", center_dist_thresh_, 0.8f);

  pnh_.param<std::string>("world_frame", w_frame_, std::string("camera_init"));
  pnh_.param<std::string>("gravity_frame", g_frame_,
                          std::string("base_gravity"));
  pnh_.param<std::string>("body_frame", b_frame_, std::string("body"));
  pnh_.param<std::string>("used_frame", used_frame_,
                          std::string("base_gravity"));
  pnh_.param<std::string>("dog_model", dog_model_, std::string("m20"));

  pnh_.param<std::string>("accumulate_cloud_topic", accumulate_cloud_topic_,
                          std::string("cloud_topic"));
  pnh_.param<std::string>("imu_topic", imu_topic_, std::string("imu"));
  pnh_.param<std::string>("passable_cloud_topic", passable_cloud_topic_,
                          std::string("passable_area"));
  pnh_.param<std::string>("impassable_cloud_topic", impassable_cloud_topic_,
                          std::string("impassable_area"));
  pnh_.param<std::string>("grid_map_topic", grid_map_topic_,
                          std::string("grid_map"));
  pnh_.param<std::string>("traversal_cost_topic", traversal_cost_topic_,
                          std::string("traversal_cost"));

  if (min_height_ > max_height_) {
    std::swap(min_height_, max_height_);
  }

  loadBodyGeometry();
  loadLidarParams();
  loadRaycastParams();
  loadPassabilityParams();
  loadElevationSolverParams();
  loadTraversalCostParams();
}

void PassableAreaNode::loadBodyGeometry() {
  std::string model_key = normalizeModelKey(dog_model_);
  if (model_key != "x30" && model_key != "m20") {
    ROS_WARN("Unknown dog_model '%s', defaulting body parameters to 'm20'",
             dog_model_.c_str());
    model_key = "m20";
  }

  const BodyGeometry defaults = defaultBodyGeometry();
  const std::string base_param = "body_params/" + model_key + "/";

  pnh_.param(base_param + "body_length", body_length_, defaults.length);
  pnh_.param(base_param + "body_width", body_width_, defaults.width);
  pnh_.param(base_param + "body_height", body_height_, defaults.height);

  ROS_INFO("Body geometry for model '%s': length=%.3f m, width=%.3f m, "
           "height=%.3f m",
           model_key.c_str(), body_length_, body_width_, body_height_);
}

void PassableAreaNode::loadLidarParams() {
  lidar_params_.clear();

  std::string model_key = normalizeModelKey(dog_model_);
  std::vector<std::string> lidar_names;
  if (model_key == "x30") {
    lidar_names = {"lidar_front_up", "lidar_front_down", "lidar_rear_up",
                   "lidar_rear_down"};
  } else {
    if (model_key != "m20") {
      ROS_WARN("Unknown dog_model '%s', falling back to 'm20' lidar params",
               dog_model_.c_str());
      model_key = "m20";
    }
    lidar_names = {"lidar_front", "lidar_rear"};
  }

  for (const auto &lidar_name : lidar_names) {
    LidarParams param;
    param.name = lidar_name;
    const std::string prefix = "lidar_params/" + model_key + "/" + lidar_name;

    std::vector<double> pos_vec{static_cast<double>(param.pos_body.x()),
                                static_cast<double>(param.pos_body.y()),
                                static_cast<double>(param.pos_body.z())};
    pnh_.param(prefix + "/pos_body", pos_vec, pos_vec);
    if (pos_vec.size() == 3) {
      param.pos_body = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
    } else {
      ROS_WARN("Invalid pos_body for %s, using defaults", lidar_name.c_str());
    }

    std::vector<double> rpy_vec{static_cast<double>(param.rpy_body_deg.x()),
                                static_cast<double>(param.rpy_body_deg.y()),
                                static_cast<double>(param.rpy_body_deg.z())};
    pnh_.param(prefix + "/rpy_body", rpy_vec, rpy_vec);
    if (rpy_vec.size() == 3) {
      param.rpy_body_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
    } else {
      ROS_WARN("Invalid rpy_body for %s, using defaults", lidar_name.c_str());
    }
    param.rpy_body_rad =
        Eigen::Vector3f(dr::degreeToRadian(param.rpy_body_deg.x()),
                        dr::degreeToRadian(param.rpy_body_deg.y()),
                        dr::degreeToRadian(param.rpy_body_deg.z()));
    param.R_mount = dr::rotationFromYPRrad(param.rpy_body_rad);

    pnh_.param(prefix + "/fov_up_deg", param.fov_up_deg, param.fov_up_deg);
    pnh_.param(prefix + "/fov_down_deg", param.fov_down_deg,
               param.fov_down_deg);
    param.fov_up_rad = dr::degreeToRadian(param.fov_up_deg);
    param.fov_down_rad = dr::degreeToRadian(param.fov_down_deg);

    pnh_.param(prefix + "/min_range", param.min_range, param.min_range);
    pnh_.param(prefix + "/max_range", param.max_range, param.max_range);

    lidar_params_.push_back(param);
  }

  ROS_INFO("Using lidar params for dog model '%s' (%zu lidars)",
           model_key.c_str(), lidar_params_.size());
}

void PassableAreaNode::loadRaycastParams() {
  const std::string base = "raycast/";
  pnh_.param(base + "enable", raycast_params_.enable, true);
  pnh_.param(base + "max_ray_distance", raycast_params_.max_ray_distance, 4.0f);
  pnh_.param(base + "max_nan_gap", raycast_params_.max_nan_gap, 1.0f);
}

void PassableAreaNode::loadElevationSolverParams() {
  const std::string base = "elevation_solver/";
  pnh_.param(base + "use_histogram_solver", solver_params_.use_histogram_solver,
             true);
  pnh_.param(base + "histogram_bins", solver_params_.histogram_bins, 16);
  pnh_.param(base + "region_enabled", solver_params_.region.enabled, false);
  pnh_.param(base + "region_min_x", solver_params_.region.min_x, 0.0f);
  pnh_.param(base + "region_max_x", solver_params_.region.max_x, 0.0f);
  pnh_.param(base + "region_min_y", solver_params_.region.min_y, 0.0f);
  pnh_.param(base + "region_max_y", solver_params_.region.max_y, 0.0f);
}

void PassableAreaNode::loadTraversalCostParams() {
  const std::string base = "traversal_cost/";
  pnh_.param(base + "enable", traversal_cost_params_.enabled, true);
  pnh_.param(base + "slope_free_deg", traversal_cost_params_.slope_free_deg,
             5.0f);
  pnh_.param(base + "slope_block_deg", traversal_cost_params_.slope_block_deg,
             30.0f);
  pnh_.param(base + "rough_free", traversal_cost_params_.rough_free, 0.02f);
  pnh_.param(base + "rough_block", traversal_cost_params_.rough_block, 0.08f);
  pnh_.param(base + "step_free", traversal_cost_params_.step_free, 0.05f);
  pnh_.param(base + "step_block", traversal_cost_params_.step_block, 0.18f);
  pnh_.param(base + "slope_weight", traversal_cost_params_.slope_weight, 0.4f);
  pnh_.param(base + "roughness_weight", traversal_cost_params_.roughness_weight,
             0.3f);
  pnh_.param(base + "step_weight", traversal_cost_params_.step_weight, 0.3f);
  pnh_.param(base + "easy_cost", traversal_cost_params_.easy_cost, 1.0f);
  pnh_.param(base + "hard_cost", traversal_cost_params_.hard_cost, 60.0f);
  pnh_.param(base + "max_cost", traversal_cost_params_.max_cost, 100.0f);
  pnh_.param(base + "curve_power", traversal_cost_params_.curve_power, 3.0f);
  pnh_.param(base + "terrain_sample_window",
             traversal_cost_params_.terrain_sample_window, 1);
  pnh_.param(base + "safe_zone_side_length",
             traversal_cost_params_.safe_zone_side_length, 1.0f);
}

void PassableAreaNode::loadPassabilityParams() {
  pnh_.param("max_drop", passability_params_.drop_threshold, 0.3f);
  pnh_.param("max_roughness", passability_params_.roughness_threshold, 0.1f);
  pnh_.param("max_slope_deg", passability_params_.max_slope_deg, 45.0f);
  pnh_.param("treat_nan_as_stiff", passability_params_.treat_nan_as_stiff,
             true);
}

std::string
PassableAreaNode::normalizeModelKey(const std::string &dog_model) const {
  std::string normalized = dog_model;
  std::transform(
      normalized.begin(), normalized.end(), normalized.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return normalized;
}

void PassableAreaNode::initialize() {
  ROS_INFO("Initializing << passable area >>");
  ROS_INFO("used frame: %s", used_frame_.c_str());
  ROS_INFO("Topic configuration:\n"
           "  cloud: %s\n"
           "  imu: %s\n"
           "  passable: %s\n"
           "  impassable: %s\n"
           "  grid_map: %s",
           accumulate_cloud_topic_.c_str(), imu_topic_.c_str(),
           passable_cloud_topic_.c_str(), impassable_cloud_topic_.c_str(),
           grid_map_topic_.c_str());

  elevationInit();

  if (enable_blind_check_) {
    lidarCoverInit();
  }

  imu_sub_ = nh_.subscribe(imu_topic_, 50, &PassableAreaNode::imuCallback, this,
                           ros::TransportHints().tcpNoDelay());
  cloud_sub_ = nh_.subscribe(accumulate_cloud_topic_, 10,
                             &PassableAreaNode::cloudCallback, this,
                             ros::TransportHints().tcpNoDelay());
  body_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("body_visual", 10);
  passable_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(passable_cloud_topic_, 10);
  impassable_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(impassable_cloud_topic_, 10);
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_, 10);
  traversal_cost_pub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>(traversal_cost_topic_, 10);
}

void PassableAreaNode::elevationInit() {
  ROS_INFO("Initializing < elevation map >");
  ele_map_ =
      std::make_unique<ElevationMap>(map_length_, map_width_, min_height_,
                                     max_height_, voxel_width_, used_frame_);
  ele_map_->setMaxInpaintPixels(max_inpaint_pixels_);
  ele_map_->setCenterPaddingParams(enable_center_padding_, center_dist_thresh_);
  ele_map_->setPassabilityParams(passability_params_);
  ele_map_->setElevationSolverParams(solver_params_);
  ele_map_->setTraversalCostParams(traversal_cost_params_);
  ele_map_->setRaycastParams(raycast_params_);
  ele_map_->setLidarParams(lidar_params_);
  traversal_cost_ = std::make_unique<TraversalCost>(
      *ele_map_, ele_map_->getTraversalCostParams());
  ele_init_ = true;
}

void PassableAreaNode::lidarCoverInit() {
  ROS_INFO("Initializing < lidar coverage >");
  lidar_cov_ = std::make_unique<LidarCoverage>(pnh_);
  lidar_cov_->initialize(dog_model_, lidar_params_);
  lidar_init_ = true;
}

void PassableAreaNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  if (used_frame_ == b_frame_)
    return; // no need if it is body frame

  Eigen::Quaternionf q(msg->orientation.w, msg->orientation.x,
                       msg->orientation.y, msg->orientation.z);
  q.normalize();

  Eigen::Matrix3f R_world2body = q.toRotationMatrix();

  float yaw = std::atan2(R_world2body(1, 0), R_world2body(0, 0));
  float pitch = std::asin(-R_world2body(2, 0));
  float roll = std::atan2(R_world2body(2, 1), R_world2body(2, 2));

  pitch = -pitch;
  Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f R_gravity2body = (Ry * Rx).toRotationMatrix();

  std::lock_guard<std::mutex> lock(imu_mutex_);
  T_g2b_.setIdentity();
  T_g2b_.linear() = R_gravity2body;
  T_g2b_.translation() = Eigen::Vector3f::Zero();
}

void PassableAreaNode::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  if (ele_init_) {
    const auto start_time = std::chrono::steady_clock::now();

    ele_map_->processPointCloud(*msg, getTransform());
    bool cost_ready = false;
    if (traversal_cost_) {
      cost_ready = traversal_cost_->updateCostLayer();
    }
    if (cost_ready) {
      ele_map_->resolveUnknownWithTraversalCost();
    }

    const auto end_time = std::chrono::steady_clock::now();
    const auto duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                              start_time)
            .count();
    if (duration_ms > 50) {
      ROS_WARN("Elevation map updated in %ld ms",
               static_cast<long>(duration_ms));
    }
  }

  if (lidar_init_) {
    lidar_cov_->processCoverage(*ele_map_, getTransform());
  }

  stamp_ = msg->header.stamp;
  bodyVisual();
  publishPassableInfo();
  publishGridMap();
  publishTraversalCost();
}

Eigen::Affine3f PassableAreaNode::getTransform() const {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return T_g2b_;
}

void PassableAreaNode::bodyVisual() {
  visualization_msgs::Marker body;

  body.header.frame_id = b_frame_;
  body.header.stamp = stamp_;
  body.type = visualization_msgs::Marker::CUBE;
  body.action = visualization_msgs::Marker::ADD;

  body.pose.position.x = 0.0;
  body.pose.position.y = 0.0;
  body.pose.position.z = 0.0;

  body.pose.orientation.x = 0.0;
  body.pose.orientation.y = 0.0;
  body.pose.orientation.z = 0.0;
  body.pose.orientation.w = 1.0;

  body.scale.x = body_length_;
  body.scale.y = body_width_;
  body.scale.z = body_height_;

  body.color.r = 0.0;
  body.color.g = 0.0;
  body.color.b = 0.8;
  body.color.a = 0.7;

  body_vis_pub_.publish(body);
}

void PassableAreaNode::publishPassableInfo() {
  const auto &cloud = ele_map_->getWorkingCloud();

  const float half_length = map_length_ * 0.5f;
  const float half_width = map_width_ * 0.5f;

  passable_cloud_.clear();
  impassable_cloud_.clear();
  expanded_cloud_.clear();

  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const auto &p = cloud[i];
    if (p.x < -half_length || p.x > half_length || p.y < -half_width ||
        p.y > half_width || p.z < min_height_ || p.z > max_height_)
      continue;

    Eigen::Vector2d pos(p.x, p.y);
    Passability step = ele_map_->getPassability(pos);
    CoverageStatus cover = ele_map_->getCoverability(pos);

    if (step == Passability::Passable) {
      passable_cloud_.push_back(p);
    } else if (step == Passability::Impassable &&
               cover == CoverageStatus::Covered) {
      impassable_cloud_.push_back(p);
    } else {
      // expanded_cloud_.push_back(p);
    }
  }

  auto finalize = [](pcl::PointCloud<pcl::PointXYZ> &c) {
    c.width = static_cast<uint32_t>(c.size());
    c.height = 1;
    c.is_dense = true;
  };

  sensor_msgs::PointCloud2 ros_passable, ros_impassable;
  finalize(passable_cloud_);
  finalize(impassable_cloud_);
  pcl::toROSMsg(passable_cloud_, ros_passable);
  pcl::toROSMsg(impassable_cloud_, ros_impassable);
  ros_passable.header.frame_id = ros_impassable.header.frame_id = used_frame_;
  ros_passable.header.stamp = ros_impassable.header.stamp = stamp_;

  passable_pub_.publish(ros_passable);
  impassable_pub_.publish(ros_impassable);
}

void PassableAreaNode::publishGridMap() {
  grid_map_msgs::GridMap ros_map;
  grid_map::GridMapRosConverter::toMessage(*ele_map_, ros_map);
  ros_map.info.header.frame_id = used_frame_;
  ros_map.info.header.stamp = stamp_;
  grid_map_pub_.publish(ros_map);
}

void PassableAreaNode::publishTraversalCost() {
  if (!traversal_cost_ || !traversal_cost_pub_)
    return;
  traversal_cost_->publish(traversal_cost_pub_, used_frame_, stamp_);
}
