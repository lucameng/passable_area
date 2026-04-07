#include "passable_area/interfaces/ros/nodes/passable_area_node.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <sys/resource.h>
#include <thread>
#include <vector>

namespace {

sensor_msgs::msg::PointCloud2 MakeCloud(int point_count, const builtin_interfaces::msg::Time &stamp) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(point_count);
  for (int i = 0; i < point_count; ++i) {
    const float x = -3.8f + 7.6f * static_cast<float>(i % 400) / 399.0f;
    const float y = -3.8f + 7.6f * static_cast<float>((i / 400) % 400) / 399.0f;
    float z = 0.03f * x;
    if (std::abs(x) < 0.8f && std::abs(y) < 0.8f) {
      z += 0.02f;
    }
    cloud.emplace_back(x, y, z);
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "base_link";
  msg.header.stamp = stamp;
  return msg;
}

nav_msgs::msg::Odometry MakeOdom(const builtin_interfaces::msg::Time &stamp) {
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "camera_init";
  msg.header.stamp = stamp;
  msg.pose.pose.orientation.w = 1.0;
  return msg;
}

double CpuSeconds() {
  struct rusage usage {};
  getrusage(RUSAGE_SELF, &usage);
  return static_cast<double>(usage.ru_utime.tv_sec) +
         static_cast<double>(usage.ru_utime.tv_usec) * 1e-6 +
         static_cast<double>(usage.ru_stime.tv_sec) +
         static_cast<double>(usage.ru_stime.tv_usec) * 1e-6;
}

double Percentile(std::vector<double> values, double p) {
  std::sort(values.begin(), values.end());
  const size_t idx = std::min(values.size() - 1,
                              static_cast<size_t>(std::floor(p * (values.size() - 1))));
  return values[idx];
}

class HarnessNode : public rclcpp::Node {
public:
  HarnessNode() : Node("passable_area_e2e_harness") {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/LOC_BODY_POINTS", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ODOM", 10);
    terrain_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/terrain_state", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onTerrain(msg); });
  }

  void publishFrame(int point_count, const builtin_interfaces::msg::Time &stamp) {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      send_times_[rclcpp::Time(stamp).nanoseconds()] = now;
    }
    odom_pub_->publish(MakeOdom(stamp));
    cloud_pub_->publish(MakeCloud(point_count, stamp));
  }

  bool waitUntilResponses(size_t count, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&]() { return latencies_ms_.size() >= count; });
  }

  std::vector<double> latencies() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latencies_ms_;
  }

private:
  void onTerrain(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = send_times_.find(stamp_ns);
    if (it == send_times_.end()) {
      return;
    }
    latencies_ms_.push_back(
        std::chrono::duration<double, std::milli>(now - it->second).count());
    send_times_.erase(it);
    cv_.notify_all();
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_sub_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::map<int64_t, std::chrono::steady_clock::time_point> send_times_;
  std::vector<double> latencies_ms_;
};

void RunBenchmarkForCount(int point_count) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("input_cloud_topic", "/LOC_BODY_POINTS");
  options.append_parameter_override("odom_topic", "/ODOM");
  options.append_parameter_override("debug.publish_grid_map", false);
  options.append_parameter_override("debug.publish_points", false);
  options.append_parameter_override("debug.publish_observability", false);
  auto passable_node = std::make_shared<passable_area::interfaces::ros::PassableAreaNode>(options);
  auto harness = std::make_shared<HarnessNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(passable_node);
  executor.add_node(harness);
  std::thread spin_thread([&executor]() { executor.spin(); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const int frame_count = 30;
  const double cpu_start = CpuSeconds();
  const auto wall_start = std::chrono::steady_clock::now();
  for (int i = 0; i < frame_count; ++i) {
    builtin_interfaces::msg::Time stamp;
    const int64_t stamp_ns = 100000000LL * i + 1LL;
    stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    harness->publishFrame(point_count, stamp);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  const bool all_received = harness->waitUntilResponses(frame_count, std::chrono::seconds(10));
  const auto wall_end = std::chrono::steady_clock::now();
  const double cpu_end = CpuSeconds();

  executor.cancel();
  spin_thread.join();

  const auto latencies = harness->latencies();
  const double avg = std::accumulate(latencies.begin(), latencies.end(), 0.0) /
                     std::max<size_t>(latencies.size(), 1U);
  const double p95 = latencies.empty() ? 0.0 : Percentile(latencies, 0.95);
  const double p99 = latencies.empty() ? 0.0 : Percentile(latencies, 0.99);
  const double wall_sec =
      std::chrono::duration<double>(wall_end - wall_start).count();
  const double cpu_ratio = wall_sec > 0.0 ? (cpu_end - cpu_start) / wall_sec * 100.0 : 0.0;
  const double drop_rate =
      100.0 * static_cast<double>(frame_count - latencies.size()) / static_cast<double>(frame_count);

  std::cout << "e2e points=" << point_count << " avg_ms=" << std::fixed << std::setprecision(2)
            << avg << " p95_ms=" << p95 << " p99_ms=" << p99
            << " drop_rate_pct=" << drop_rate << " cpu_pct=" << cpu_ratio
            << " received_all=" << std::boolalpha << all_received << '\n';
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  RunBenchmarkForCount(80000);
  RunBenchmarkForCount(160000);
  rclcpp::shutdown();
  return 0;
}
