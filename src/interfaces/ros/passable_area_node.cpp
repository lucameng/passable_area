#include "passable_area/interfaces/ros/passable_area_node.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <chrono>
#include <cstdarg>

namespace passable_area::interfaces::ros {
namespace {

constexpr const char *kDefaultDrLoggerName = "passable_area";
constexpr const char *kDefaultDrLoggerLogPath = "/var/opt/robot/log";
constexpr const char *kDefaultDrLoggerPropertiesPath =
    "/var/opt/robot/conf/log.properties";

Eigen::Quaternionf YawOnlyQuaternion(const Eigen::Quaternionf &orientation) {
  return Eigen::Quaternionf(
      Eigen::AngleAxisf(passable_area::core::YawFromQuaternion(orientation),
                        Eigen::Vector3f::UnitZ()));
}

} // namespace

PassableAreaNode::PassableAreaNode(const rclcpp::NodeOptions &options)
    : Node("passable_area", options),
      node_params_(RosParamLoader{}.load(*this)), config_(node_params_.config),
      processor_(config_), topic_config_(node_params_.topics) {
  initDrLogger();
  result_publishers_.initialize(*this, topic_config_);
  debug_publishers_.initialize(*this, topic_config_, config_.debug);
  status_code_manager_.initialize(*this, topic_config_, node_logger_);
  watchdog_.initialize(*this, topic_config_.input_cloud_topic,
                       topic_config_.odom_topic, node_logger_);
  perf_stats_.initialize(*this, node_logger_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  cloud_sub_.subscribe(this, topic_config_.input_cloud_topic,
                       sensor_qos.get_rmw_qos_profile());
  odom_sub_.subscribe(this, topic_config_.odom_topic,
                      sensor_qos.get_rmw_qos_profile());
  cloud_sub_.registerCallback(std::bind(&PassableAreaNode::onCloudObserved,
                                        this, std::placeholders::_1));
  odom_sub_.registerCallback(std::bind(&PassableAreaNode::onOdomObserved, this,
                                       std::placeholders::_1));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(topic_config_.sync_queue_size), cloud_sub_, odom_sub_);
  sync_->registerCallback(std::bind(&PassableAreaNode::onSynced, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
}

bool PassableAreaNode::initDrLogger() {
  const auto options =
      resolveDrLoggerOptions(kDefaultDrLoggerName, kDefaultDrLoggerLogPath,
                             kDefaultDrLoggerPropertiesPath);
  log_bridge_ = std::make_shared<LogBridge>(options);
  node_logger_ = std::make_shared<NodeLogger>(
      log_bridge_,
      [this](NodeLogger::Level level, const std::string &message) {
        switch (level) {
        case NodeLogger::Level::kDebug:
          RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
          break;
        case NodeLogger::Level::kInfo:
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
          break;
        case NodeLogger::Level::kWarn:
          RCLCPP_WARN(get_logger(), "%s", message.c_str());
          break;
        case NodeLogger::Level::kError:
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
          break;
        case NodeLogger::Level::kFatal:
          RCLCPP_FATAL(get_logger(), "%s", message.c_str());
          break;
        }
      },
      [this]() { return get_clock()->now().seconds(); });

  std::string error_message;
  const bool dr_enabled = log_bridge_->init(error_message);
  if (!dr_enabled && !error_message.empty()) {
    RCLCPP_ERROR(get_logger(),
                 "DrLogger init failed, fallback to ROS-only logging: %s",
                 error_message.c_str());
    return false;
  }
  if (!dr_enabled) {
    RCLCPP_INFO(get_logger(),
                "DrLogger disabled by env %s, using ROS-only logging.",
                "PASSABLE_DR_LOGGER_ENABLE");
    return false;
  }
  node_logger_->log(NodeLogger::Level::kInfo,
                    "DrLogger enabled, ROS sink disabled.");
  return true;
}

void PassableAreaNode::logDebug(const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logV(NodeLogger::Level::kDebug, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_DEBUG(get_logger(), "%s", message.c_str());
}

void PassableAreaNode::logInfo(const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logV(NodeLogger::Level::kInfo, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
}

void PassableAreaNode::logWarn(const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logV(NodeLogger::Level::kWarn, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

void PassableAreaNode::logWarnThrottle(uint64_t throttle_ms, const char *key,
                                       const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logWarnThrottleV(static_cast<double>(throttle_ms) / 1000.0,
                                   key, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), throttle_ms, "%s",
                       message.c_str());
}

void PassableAreaNode::logError(const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logV(NodeLogger::Level::kError, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
}

void PassableAreaNode::logFatal(const char *format, ...) {
  va_list args;
  va_start(args, format);
  if (node_logger_) {
    node_logger_->logV(NodeLogger::Level::kFatal, format, args);
    va_end(args);
    return;
  }
  const std::string message = formatLogMessage(format, args);
  va_end(args);
  RCLCPP_FATAL(get_logger(), "%s", message.c_str());
}

void PassableAreaNode::onCloudObserved(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  (void)msg;
  status_code_manager_.markCloud(now());
  watchdog_.markCloud(now());
  perf_stats_.observeCloud();
}

void PassableAreaNode::onOdomObserved(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  (void)msg;
  status_code_manager_.markOdom(now());
  watchdog_.markOdom(now());
  perf_stats_.observeOdom();
}

void PassableAreaNode::onSynced(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
  status_code_manager_.markSynced(now());
  watchdog_.markSynced(now());
  auto start = std::chrono::steady_clock::now();

  try {
    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!point_converter_.fromRos(*cloud_msg, cloud)) {
      status_code_manager_.markOutputFailure();
      logWarnThrottle(2000, "pointcloud_ros_convert_failed",
                      "PointCloud2 conversion failed");
      return;
    }
    if (!odom_converter_.fromRos(*odom_msg, pose)) {
      status_code_manager_.markOutputFailure();
      logWarnThrottle(2000, "odom_ros_convert_failed",
                      "Odometry conversion failed");
      return;
    }

    passable_area::core::FrameInput input;
    input.stamp = rclcpp::Time(cloud_msg->header.stamp).nanoseconds();
    input.base_pose_in_odom = pose;
    input.input_cloud_in_base = std::move(cloud);

    const auto output = processor_.update(input);
    if (!output.valid) {
      status_code_manager_.markOutputFailure();
      logWarnThrottle(2000, "processor_dropped_frame",
                      "Processor dropped frame");
      return;
    }

    std_msgs::msg::Header base_gravity_header;
    base_gravity_header.stamp = cloud_msg->header.stamp;
    base_gravity_header.frame_id = config_.base_gravity_frame;
    result_publishers_.publish(output, base_gravity_header);
    debug_publishers_.publish(output, base_gravity_header);
    publishBaseGravityTransform(pose, cloud_msg->header.stamp);
    status_code_manager_.markOutputSuccess();

    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    perf_stats_.record(ms);
  } catch (const std::exception &e) {
    status_code_manager_.markFatalRuntimeException(e.what());
    logError("Unhandled exception in synced callback: %s", e.what());
  } catch (...) {
    status_code_manager_.markFatalRuntimeException("unknown exception");
    logError("Unhandled non-std exception in synced callback");
  }
}

void PassableAreaNode::publishBaseGravityTransform(
    const passable_area::core::Pose3D &base_pose_in_odom,
    const builtin_interfaces::msg::Time &stamp) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = config_.odom_frame;
  transform.child_frame_id = config_.base_gravity_frame;
  transform.transform.translation.x = base_pose_in_odom.position.x();
  transform.transform.translation.y = base_pose_in_odom.position.y();
  transform.transform.translation.z = base_pose_in_odom.position.z();

  const Eigen::Quaternionf yaw_only =
      YawOnlyQuaternion(base_pose_in_odom.orientation);
  transform.transform.rotation.x = yaw_only.x();
  transform.transform.rotation.y = yaw_only.y();
  transform.transform.rotation.z = yaw_only.z();
  transform.transform.rotation.w = yaw_only.w();
  tf_broadcaster_->sendTransform(transform);
}

} // namespace passable_area::interfaces::ros
