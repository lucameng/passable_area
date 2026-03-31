#include "passable_area_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>

PassableAreaNode::PassableAreaNode()
    : Node("passable_area"), ele_init_(false), lidar_init_(false),
      enable_blind_check_(declare_parameter("enable_blind_check", false)),
      map_length_(declare_parameter("map_length", 10.0f)),
      map_width_(declare_parameter("map_width", 10.0f)),
      min_height_(declare_parameter("map_height_min", -1.5f)),
      max_height_(declare_parameter("map_height_max", 1.5f)),
      voxel_width_(declare_parameter("voxel_size", 0.1f)),
      max_inpaint_pixels_(declare_parameter<int>("max_inpaint_pixels", 200)),
      enable_center_padding_(declare_parameter("enable_center_padding", true)),
      center_dist_thresh_(declare_parameter("center_dist_thresh", 0.8f)),
      w_frame_(declare_parameter<std::string>("world_frame", "camera_init")),
      g_frame_(declare_parameter<std::string>("gravity_frame", "base_gravity")),
      b_frame_(declare_parameter<std::string>("body_frame", "base_link")),
      dog_model_(declare_parameter<std::string>("dog_model", "m20")),
      input_cloud_topic_(declare_parameter<std::string>("input_cloud_topic",
                                                        "/LOC_BODY_POINTS")),
      odom_topic_(declare_parameter<std::string>("odom_topic", "/ODOM")),
      imu_topic_(declare_parameter<std::string>("imu_topic", "imu")),
      passable_cloud_topic_(declare_parameter<std::string>(
          "passable_cloud_topic", "passable_area")),
      impassable_cloud_topic_(declare_parameter<std::string>(
          "impassable_cloud_topic", "impassable_area")),
      passable_status_code_topic_(declare_parameter<std::string>(
          "passable_status_code_topic", "/passable_status_code")),
      grid_map_topic_(
          declare_parameter<std::string>("grid_map_topic", "grid_map")),
      traversal_cost_topic_(declare_parameter<std::string>(
          "traversal_cost_topic", "traversal_cost")),
      body_length_(0.0f), body_width_(0.0f), body_height_(0.0f),
      last_odom_msg_time_(0, 0, get_clock()->get_clock_type()),
      last_cloud_msg_time_(0, 0, get_clock()->get_clock_type()),
      last_synced_msg_time_(0, 0, get_clock()->get_clock_type()),
      perf_window_start_time_(0, 0, get_clock()->get_clock_type()),
      odom_received_(false), cloud_received_(false), synced_received_(false),
      sync_queue_size_(10), perf_cloud_count_(0), perf_odom_count_(0),
      perf_synced_count_(0), perf_total_ms_sum_(0.0), perf_total_ms_max_(0.0) {
  if (min_height_ > max_height_) {
    std::swap(min_height_, max_height_);
  }
  loadBodyGeometry();
  loadLidarParams();
  loadRaycastParams();
  loadDownsampleParams();
  loadSyncParams();
  loadPassabilityParams();
  loadElevationSolverParams();
  loadTraversalCostParams();
}

void PassableAreaNode::loadBodyGeometry() {
  std::string model_key = normalizeModelKey(dog_model_);
  if (model_key != "x30" && model_key != "m20") {
    RCLCPP_WARN(get_logger(),
                "Unknown dog_model '%s', defaulting body parameters to 'm20'",
                dog_model_.c_str());
    model_key = "m20";
  }

  const BodyGeometry defaults = defaultBodyGeometry();
  const std::string base_param = "body_params." + model_key + ".";

  body_length_ =
      declare_parameter<float>(base_param + "body_length", defaults.length);
  body_width_ =
      declare_parameter<float>(base_param + "body_width", defaults.width);
  body_height_ =
      declare_parameter<float>(base_param + "body_height", defaults.height);

  RCLCPP_INFO(get_logger(),
              "Body geometry for model '%s': length=%.3f m, width=%.3f m, "
              "height=%.3f m",
              model_key.c_str(), body_length_, body_width_, body_height_);
}

void PassableAreaNode::loadElevationSolverParams() {
  const std::string base = "elevation_solver.";
  solver_params_.use_histogram_solver =
      declare_parameter<bool>(base + "use_histogram_solver", true);
  solver_params_.histogram_bins =
      declare_parameter<int>(base + "histogram_bins", 16);
  solver_params_.region.enabled =
      declare_parameter<bool>(base + "region_enabled", false);
  solver_params_.region.min_x =
      declare_parameter<float>(base + "region_min_x", 0.0f);
  solver_params_.region.max_x =
      declare_parameter<float>(base + "region_max_x", 0.0f);
  solver_params_.region.min_y =
      declare_parameter<float>(base + "region_min_y", 0.0f);
  solver_params_.region.max_y =
      declare_parameter<float>(base + "region_max_y", 0.0f);
}

void PassableAreaNode::loadTraversalCostParams() {
  traversal_cost_params_.enabled =
      declare_parameter("traversal_cost.enable", true);
  traversal_cost_params_.slope_free_deg =
      declare_parameter("traversal_cost.slope_free_deg", 5.0f);
  traversal_cost_params_.slope_block_deg =
      declare_parameter("traversal_cost.slope_block_deg", 30.0f);
  traversal_cost_params_.rough_free =
      declare_parameter("traversal_cost.rough_free", 0.02f);
  traversal_cost_params_.rough_block =
      declare_parameter("traversal_cost.rough_block", 0.08f);
  traversal_cost_params_.step_free =
      declare_parameter("traversal_cost.step_free", 0.05f);
  traversal_cost_params_.step_block =
      declare_parameter("traversal_cost.step_block", 0.18f);
  traversal_cost_params_.slope_weight =
      declare_parameter("traversal_cost.slope_weight", 0.4f);
  traversal_cost_params_.roughness_weight =
      declare_parameter("traversal_cost.roughness_weight", 0.3f);
  traversal_cost_params_.step_weight =
      declare_parameter("traversal_cost.step_weight", 0.3f);
  traversal_cost_params_.easy_cost =
      declare_parameter("traversal_cost.easy_cost", 0.0f);
  traversal_cost_params_.hard_cost =
      declare_parameter("traversal_cost.hard_cost", 90.0f);
  traversal_cost_params_.max_cost =
      declare_parameter("traversal_cost.max_cost", 100.0f);
  traversal_cost_params_.missing_cost =
      declare_parameter("traversal_cost.missing_cost", 20.0f);
  traversal_cost_params_.curve_power =
      declare_parameter("traversal_cost.curve_power", 1.0f);
  traversal_cost_params_.low_cost_filter_ratio =
      declare_parameter("traversal_cost.low_cost_filter_ratio", 0.0f);
  traversal_cost_params_.terrain_sample_window =
      declare_parameter<int>("traversal_cost.terrain_sample_window", 1);
  traversal_cost_params_.safe_zone_side_length =
      declare_parameter("traversal_cost.safe_zone_side_length", 1.0f);
}

void PassableAreaNode::loadRaycastParams() {
  raycast_params_.enable = declare_parameter("raycast.enable", true);
  raycast_params_.max_ray_distance =
      declare_parameter("raycast.max_ray_distance", 4.0f);
  raycast_params_.max_nan_gap = declare_parameter("raycast.max_nan_gap", 1.0f);
}

void PassableAreaNode::loadDownsampleParams() {
  downsample_params_.enable = declare_parameter("downsample.enable", true);
  downsample_params_.voxel_size =
      declare_parameter("downsample.voxel_size", voxel_width_);
  downsample_params_.voxel_size =
      std::max(1e-3f, downsample_params_.voxel_size);
}

void PassableAreaNode::loadSyncParams() {
  sync_queue_size_ = declare_parameter<int>("sync.queue_size", 10);
  sync_queue_size_ = std::max(2, sync_queue_size_);
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
      RCLCPP_WARN(get_logger(),
                  "Unknown dog_model '%s', falling back to 'm20' lidar params",
                  dog_model_.c_str());
      model_key = "m20";
    }
    lidar_names = {"lidar_front", "lidar_rear"};
  }

  for (const auto &lidar_name : lidar_names) {
    LidarParams param;
    param.name = lidar_name;
    const std::string prefix = "lidar_params." + model_key + "." + lidar_name;

    std::vector<double> pos_vec;
    declare_parameter(prefix + ".pos_body",
                      std::vector<double>{param.pos_body.x(),
                                          param.pos_body.y(),
                                          param.pos_body.z()});
    get_parameter(prefix + ".pos_body", pos_vec);
    if (pos_vec.size() == 3) {
      param.pos_body = Eigen::Vector3f(pos_vec[0], pos_vec[1], pos_vec[2]);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid pos_body for %s, using defaults",
                  lidar_name.c_str());
    }

    std::vector<double> rpy_vec;
    declare_parameter(prefix + ".rpy_body",
                      std::vector<double>{param.rpy_body_deg.x(),
                                          param.rpy_body_deg.y(),
                                          param.rpy_body_deg.z()});
    get_parameter(prefix + ".rpy_body", rpy_vec);
    if (rpy_vec.size() == 3) {
      param.rpy_body_deg = Eigen::Vector3f(rpy_vec[0], rpy_vec[1], rpy_vec[2]);
      param.rpy_body_rad = Eigen::Vector3f(dr::degreeToRadian(rpy_vec[0]),
                                           dr::degreeToRadian(rpy_vec[1]),
                                           dr::degreeToRadian(rpy_vec[2]));
      param.R_mount = dr::rotationFromYPRrad(param.rpy_body_rad);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid rpy_body for %s, using defaults",
                  lidar_name.c_str());
    }

    declare_parameter(prefix + ".fov_up_deg", param.fov_up_deg);
    declare_parameter(prefix + ".fov_down_deg", param.fov_down_deg);
    get_parameter(prefix + ".fov_up_deg", param.fov_up_deg);
    get_parameter(prefix + ".fov_down_deg", param.fov_down_deg);
    param.fov_up_rad = dr::degreeToRadian(param.fov_up_deg);
    param.fov_down_rad = dr::degreeToRadian(param.fov_down_deg);

    declare_parameter(prefix + ".min_range", param.min_range);
    declare_parameter(prefix + ".max_range", param.max_range);
    get_parameter(prefix + ".min_range", param.min_range);
    get_parameter(prefix + ".max_range", param.max_range);

    lidar_params_.push_back(param);
  }
  RCLCPP_INFO(get_logger(),
              "Using lidar params for dog model '%s' (%zu lidars)",
              model_key.c_str(), lidar_params_.size());
}

void PassableAreaNode::loadPassabilityParams() {
  passability_params_.drop_threshold = declare_parameter("max_drop", 0.3f);
  passability_params_.roughness_threshold =
      declare_parameter("max_roughness", 0.1f);
  passability_params_.max_slope_deg = declare_parameter("max_slope_deg", 45.0f);
  passability_params_.treat_nan_as_stiff =
      declare_parameter("treat_nan_as_stiff", true);
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
  RCLCPP_INFO(get_logger(), "Initializing << passable area >>");
  RCLCPP_INFO(get_logger(), "gravity frame: %s", g_frame_.c_str());
  RCLCPP_INFO(get_logger(),
              "Topic configuration:\n"
              "  input_cloud: %s\n"
              "  odom: %s\n"
              "  imu: %s (reserved, ignored in current pipeline)\n"
              "  passable: %s\n"
              "  impassable: %s\n"
              "  status_code: %s\n"
              "  grid_map: %s\n"
              "  sync.queue_size: %d",
              input_cloud_topic_.c_str(), odom_topic_.c_str(),
              imu_topic_.c_str(), passable_cloud_topic_.c_str(),
              impassable_cloud_topic_.c_str(),
              passable_status_code_topic_.c_str(), grid_map_topic_.c_str(),
              sync_queue_size_);
  elevationInit();

  if (enable_blind_check_) {
    lidarCoverInit();
  }
  body_vis_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("body_visual", 10);
  passable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      passable_cloud_topic_, 10);
  impassable_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      impassable_cloud_topic_, 10);
  passable_status_code_pub_ =
      create_publisher<std_msgs::msg::Int32>(passable_status_code_topic_, 10);
  // expanded_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
  //     "expanded_cloud", 10);
  grid_map_pub_ =
      create_publisher<grid_map_msgs::msg::GridMap>(grid_map_topic_, 10);
  traversal_cost_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>(traversal_cost_topic_, 10);

  // Keep QoS explicit: PointCloud2 commonly uses sensor-data / best-effort and
  // mismatched QoS can stall message_filters in ROS 2.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  cloud_sub_.subscribe(this, input_cloud_topic_,
                       sensor_qos.get_rmw_qos_profile());
  odom_sub_.subscribe(this, odom_topic_, sensor_qos.get_rmw_qos_profile());
  // In ROS 2 Humble, message_filters::Subscriber is also a SimpleFilter, so
  // registerCallback() can coexist with Synchronizer and still share the same
  // underlying subscription. This preserves single-topic single-subscription.
  cloud_sub_.registerCallback(
      std::bind(&PassableAreaNode::observeCloud, this, std::placeholders::_1));
  odom_sub_.registerCallback(std::bind(&PassableAreaNode::observeOdometry, this,
                                       std::placeholders::_1));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&PassableAreaNode::syncedCallback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
  startDataWatchdog();
}

void PassableAreaNode::startDataWatchdog() {
  using namespace std::chrono_literals;

  data_watchdog_timer_ = create_wall_timer(
      2s, std::bind(&PassableAreaNode::checkDataHealth, this));
}

void PassableAreaNode::checkDataHealth() {
  const auto now_time = now();
  const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(2.0);

  rclcpp::Time last_odom;
  rclcpp::Time last_cloud;
  rclcpp::Time last_synced;
  bool odom_received = false;
  bool cloud_received = false;
  bool synced_received = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_odom = last_odom_msg_time_;
    last_cloud = last_cloud_msg_time_;
    last_synced = last_synced_msg_time_;
    odom_received = odom_received_;
    cloud_received = cloud_received_;
    synced_received = synced_received_;
  }

  if (!odom_received) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for odometry on topic '%s' (no data received)",
        odom_topic_.c_str());
  } else if ((now_time - last_odom) > timeout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No odometry received for %.1f s on topic '%s'",
                         (now_time - last_odom).seconds(), odom_topic_.c_str());
  }

  if (!cloud_received) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for input cloud on topic '%s' (no data received)",
        input_cloud_topic_.c_str());
  } else if ((now_time - last_cloud) > timeout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No input cloud received for %.1f s on topic '%s'",
                         (now_time - last_cloud).seconds(),
                         input_cloud_topic_.c_str());
  }

  const bool cloud_fresh = cloud_received && (now_time - last_cloud) <= timeout;
  const bool odom_fresh = odom_received && (now_time - last_odom) <= timeout;
  const bool pairing_stalled =
      cloud_fresh && odom_fresh &&
      (!synced_received || (now_time - last_synced) > timeout);
  if (pairing_stalled) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Input cloud and odometry are arriving, but "
                         "synchronized pairs are stalled");
  }

  updateAndMaybeLogPerf();
}

void PassableAreaNode::elevationInit() {
  RCLCPP_INFO(get_logger(), "Initializing < elevation map >");
  ele_map_ = std::make_unique<ElevationMap>(
      map_length_, map_width_, min_height_, max_height_, voxel_width_, g_frame_,
      get_logger());
  ele_map_->setMaxInpaintPixels(max_inpaint_pixels_);
  ele_map_->setCenterPaddingParams(enable_center_padding_, center_dist_thresh_);
  ele_map_->setPassabilityParams(passability_params_);
  ele_map_->setElevationSolverParams(solver_params_);
  ele_map_->setTraversalCostParams(traversal_cost_params_);
  ele_map_->setRaycastParams(raycast_params_);
  ele_map_->setLidarParams(lidar_params_);
  traversal_cost_ = std::make_unique<TraversalCost>(
      *ele_map_, ele_map_->getTraversalCostParams(), get_logger());
  ele_init_ = true;
}

void PassableAreaNode::lidarCoverInit() {
  RCLCPP_INFO(get_logger(), "Initializing < lidar coverage >");
  lidar_cov_ = std::make_unique<LidarCoverage>(shared_from_this());
  lidar_cov_->initialize(dog_model_, lidar_params_);
  lidar_init_ = true;
}

void PassableAreaNode::observeCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const auto recv_time = now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_cloud_msg_time_ = recv_time;
    cloud_received_ = true;
    ++perf_cloud_count_;
  }
}

void PassableAreaNode::observeOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  const auto recv_time = now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_odom_msg_time_ = recv_time;
    odom_received_ = true;
    ++perf_odom_count_;
  }
}

void PassableAreaNode::updateAndMaybeLogPerf() {
  const auto current_host_time = now();
  double fps = 0.0;
  double cloud_hz = 0.0;
  double odom_hz = 0.0;
  double avg_ms = 0.0;
  double max_ms = 0.0;
  bool should_log = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (perf_window_start_time_.nanoseconds() == 0) {
      perf_window_start_time_ = current_host_time;
      return;
    }

    const double window_sec =
        std::max((current_host_time - perf_window_start_time_).seconds(), 1e-3);
    if (window_sec < 1.0) {
      return;
    }

    fps = static_cast<double>(perf_synced_count_) / window_sec;
    cloud_hz = static_cast<double>(perf_cloud_count_) / window_sec;
    odom_hz = static_cast<double>(perf_odom_count_) / window_sec;
    if (perf_synced_count_ > 0) {
      avg_ms = perf_total_ms_sum_ / static_cast<double>(perf_synced_count_);
      max_ms = perf_total_ms_max_;
    }

    perf_window_start_time_ = current_host_time;
    perf_cloud_count_ = 0;
    perf_odom_count_ = 0;
    perf_synced_count_ = 0;
    perf_total_ms_sum_ = 0.0;
    perf_total_ms_max_ = 0.0;
    should_log = true;
  }

  if (!should_log) {
    return;
  }

  RCLCPP_INFO(get_logger(),
              "pass fps=%.1f, input_hz(cloud/odom)=%.1f/%.1f, "
              "total(avg/max)=%.2f/%.2f ms",
              fps, cloud_hz, odom_hz, avg_ms, max_ms);
}

bool PassableAreaNode::buildGravityTransformFromOdom(
    const nav_msgs::msg::Odometry &msg, Eigen::Affine3f &T_g2b) const {
  Eigen::Quaternionf q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-6f) {
    return false;
  }
  q.normalize();

  Eigen::Matrix3f R_world2body = q.toRotationMatrix();
  float pitch = std::asin(-std::clamp(R_world2body(2, 0), -1.0f, 1.0f));
  float roll = std::atan2(R_world2body(2, 1), R_world2body(2, 2));
  Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf Ry(-pitch, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f R_gravity2body = (Ry * Rx).toRotationMatrix();

  T_g2b.setIdentity();
  T_g2b.linear() = R_gravity2body;
  T_g2b.translation() = Eigen::Vector3f::Zero();
  return true;
}

bool PassableAreaNode::preprocessCloud(
    const sensor_msgs::msg::PointCloud2 &msg, const Eigen::Affine3f &T_g2b,
    sensor_msgs::msg::PointCloud2 &processed_msg) {
  if (msg.header.frame_id != b_frame_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping input cloud with frame '%s'; expected body_frame '%s'",
        msg.header.frame_id.c_str(), b_frame_.c_str());
    return false;
  }

  const auto has_field = [&](const char *name) {
    return std::any_of(msg.fields.begin(), msg.fields.end(),
                       [&](const auto &field) { return field.name == name; });
  };
  if (msg.width == 0 || msg.height == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropping malformed PointCloud2 from '%s': invalid "
                         "width/height (%u/%u)",
                         input_cloud_topic_.c_str(), msg.width, msg.height);
    return false;
  }
  if (msg.point_step == 0 || msg.row_step == 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropping malformed PointCloud2 from '%s': invalid "
                         "point_step/row_step (%u/%u)",
                         input_cloud_topic_.c_str(), msg.point_step,
                         msg.row_step);
    return false;
  }
  const std::size_t min_row_step = static_cast<std::size_t>(msg.width) *
                                   static_cast<std::size_t>(msg.point_step);
  if (static_cast<std::size_t>(msg.row_step) < min_row_step) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropping malformed PointCloud2 from '%s': row_step "
                         "too small (%u < %zu)",
                         input_cloud_topic_.c_str(), msg.row_step,
                         min_row_step);
    return false;
  }
  const std::size_t expected_data_size =
      static_cast<std::size_t>(msg.row_step) *
      static_cast<std::size_t>(msg.height);
  // Smaller than row_step * height is definitely malformed. Larger is tolerated
  // here to avoid over-rejecting padded layouts; PCL parsing remains guarded
  // below.
  if (msg.data.size() < expected_data_size) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed PointCloud2 from '%s': data too small (%zu < %zu)",
        input_cloud_topic_.c_str(), msg.data.size(), expected_data_size);
    return false;
  }
  if (msg.fields.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed PointCloud2 from '%s': empty fields",
        input_cloud_topic_.c_str());
    return false;
  }
  if (!has_field("x") || !has_field("y") || !has_field("z")) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed PointCloud2 from '%s': missing x/y/z fields",
        input_cloud_topic_.c_str());
    return false;
  }

  try {
    pcl::PointCloud<pcl::PointXYZ> input_cloud;
    pcl::fromROSMsg(msg, input_cloud);

    pcl::PointCloud<pcl::PointXYZ> gravity_cloud;
    const Eigen::Affine3f T_b2g = T_g2b.inverse();
    pcl::transformPointCloud(input_cloud, gravity_cloud, T_b2g);

    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    if (downsample_params_.enable) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(gravity_cloud.makeShared());
      voxel_filter.setLeafSize(downsample_params_.voxel_size,
                               downsample_params_.voxel_size,
                               downsample_params_.voxel_size);
      voxel_filter.filter(filtered_cloud);
    } else {
      filtered_cloud.swap(gravity_cloud);
    }

    pcl::toROSMsg(filtered_cloud, processed_msg);
    processed_msg.header = msg.header;
    processed_msg.header.frame_id = g_frame_;
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to preprocess cloud from '%s': %s",
                         input_cloud_topic_.c_str(), e.what());
    return false;
  } catch (...) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to preprocess cloud from '%s': unknown exception",
        input_cloud_topic_.c_str());
    return false;
  }
}

void PassableAreaNode::syncedCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom) {
  const auto start_time = std::chrono::steady_clock::now();

  Eigen::Affine3f T_g2b = Eigen::Affine3f::Identity();
  if (!buildGravityTransformFromOdom(*odom, T_g2b)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Received invalid odometry orientation on topic '%s'",
                         odom_topic_.c_str());
    return;
  }

  sensor_msgs::msg::PointCloud2 processed_cloud;
  if (!preprocessCloud(*cloud, T_g2b, processed_cloud)) {
    return;
  }

  const auto processed_time = now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_synced_msg_time_ = processed_time;
    synced_received_ = true;
  }

  if (ele_init_) {
    ele_map_->processPointCloud(processed_cloud, T_g2b);
    bool cost_ready = false;
    if (traversal_cost_) {
      cost_ready = traversal_cost_->updateCostLayer();
    }
    if (cost_ready) {
      ele_map_->resolveUnknownWithTraversalCost();
    }
  }

  if (lidar_init_) {
    lidar_cov_->processCoverage(*ele_map_, T_g2b);
  }

  stamp_ = processed_cloud.header.stamp;
  bodyVisual();
  publishPassableInfo();
  publishGridMap();
  publishTraversalCost();

  const auto end_time = std::chrono::steady_clock::now();
  const double total_ms =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
          end_time - start_time)
          .count();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ++perf_synced_count_;
    perf_total_ms_sum_ += total_ms;
    perf_total_ms_max_ = std::max(perf_total_ms_max_, total_ms);
  }
}

void PassableAreaNode::bodyVisual() {
  visualization_msgs::msg::Marker body;

  body.header.frame_id = b_frame_;
  body.header.stamp = stamp_;
  body.type = visualization_msgs::msg::Marker::CUBE;
  body.action = visualization_msgs::msg::Marker::ADD;

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

  body_vis_pub_->publish(body);
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

  sensor_msgs::msg::PointCloud2 ros_passable, ros_impassable;
  finalize(passable_cloud_);
  finalize(impassable_cloud_);
  pcl::toROSMsg(passable_cloud_, ros_passable);
  pcl::toROSMsg(impassable_cloud_, ros_impassable);
  ros_passable.header.frame_id = ros_impassable.header.frame_id = g_frame_;
  ros_passable.header.stamp = ros_impassable.header.stamp = stamp_;

  std_msgs::msg::Int32 status_code_msg;
  status_code_msg.data = 100;
  passable_status_code_pub_->publish(status_code_msg);
  passable_pub_->publish(ros_passable);
  impassable_pub_->publish(ros_impassable);

  // visualize for debug
  // sensor_msgs::msg::PointCloud2 ros_expanded;
  // finalize(expanded_cloud_);
  // pcl::toROSMsg(expanded_cloud_, ros_expanded);
  // ros_expanded.header.frame_id = g_frame_;
  // ros_expanded.header.stamp = stamp_;
  // expanded_pub_->publish(ros_expanded);
}

void PassableAreaNode::publishGridMap() {
  auto ros_map_ptr = grid_map::GridMapRosConverter::toMessage(*ele_map_);

  ros_map_ptr->header.frame_id = g_frame_;
  ros_map_ptr->header.stamp = stamp_;

  grid_map_pub_->publish(*ros_map_ptr);
}

void PassableAreaNode::publishTraversalCost() {
  if (!traversal_cost_ || !traversal_cost_pub_)
    return;
  traversal_cost_->publish(traversal_cost_pub_, g_frame_, stamp_);
}
