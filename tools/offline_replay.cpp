#include "passable_area/interfaces/ros/converters/odom_converter.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/passable_area.hpp"
#include "passable_area/tools/false_obstacle_analyzer.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

namespace {

using passable_area::core::FrameInput;
using passable_area::core::FrameOutput;
using passable_area::core::ObservabilityState;
using passable_area::core::PassabilityState;

passable_area::core::Config MakeConfig() {
  passable_area::core::Config config;
  config.map.length = 8.0f;
  config.map.width = 8.0f;
  config.map.resolution = 0.1f;
  config.preprocess.enable_downsample = false;
  config.observability.sector_count = 72;
  config.observability.min_points_per_sector = 12;
  config.observability.dropout_sector_gap_threshold = 8;
  config.observability.min_support_confidence = 0.2f;
  config.persistence.support_persistence_frames = 5;
  config.observability.stale_to_unknown_time = 0.8f;
  config.geometry.max_support_slope_deg = 28.0f;
  config.geometry.max_step_up = 0.22f;
  config.geometry.max_step_down = 0.28f;
  config.geometry.max_support_roughness = 0.08f;
  config.geometry.min_clearance = 0.35f;
  return config;
}

FrameInput MakeBaseFrame(int64_t stamp) {
  FrameInput input;
  input.stamp = stamp;
  input.base_pose_in_odom.position = Eigen::Vector3f::Zero();
  input.base_pose_in_odom.orientation = Eigen::Quaternionf::Identity();
  return input;
}

FrameInput MakeFlatScene(int64_t stamp = 1) {
  auto input = MakeBaseFrame(stamp);
  for (float x = -3.0f; x <= 3.0f; x += 0.1f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeSlopeScene(int64_t stamp) {
  auto input = MakeBaseFrame(stamp);
  for (float x = -3.0f; x <= 3.0f; x += 0.1f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.08f * x});
    }
  }
  return input;
}

FrameInput MakeStairScene(int64_t stamp) {
  auto input = MakeBaseFrame(stamp);
  for (float x = -2.5f; x <= 2.5f; x += 0.06f) {
    const float z = 0.09f * std::floor((x + 2.5f) / 0.45f);
    for (float y = -1.5f; y <= 1.5f; y += 0.08f) {
      input.input_cloud_in_base.push_back({x, y, z});
    }
  }
  return input;
}

FrameInput MakeLowCeilingScene(int64_t stamp) {
  auto input = MakeFlatScene(stamp);
  for (float x = -1.0f; x <= 1.0f; x += 0.1f) {
    for (float y = -1.0f; y <= 1.0f; y += 0.1f) {
      input.input_cloud_in_base.push_back({x, y, 0.22f});
    }
  }
  return input;
}

FrameInput MakeRearNormalOpenScene(int64_t stamp) {
  auto input = MakeBaseFrame(stamp);
  for (float x = 0.0f; x <= 3.0f; x += 0.08f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.08f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  for (float x = -3.0f; x <= -0.8f; x += 0.3f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.3f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeRearDropoutScene(int64_t stamp) {
  auto input = MakeBaseFrame(stamp);
  for (float x = 0.0f; x <= 3.0f; x += 0.06f) {
    for (float y = -2.0f; y <= 2.0f; y += 0.06f) {
      input.input_cloud_in_base.push_back({x, y, 0.0f});
    }
  }
  return input;
}

FrameInput MakeLocalHoleScene(int64_t stamp) {
  auto input = MakeFlatScene(stamp);
  input.input_cloud_in_base.erase(
      std::remove_if(input.input_cloud_in_base.begin(), input.input_cloud_in_base.end(),
                     [](const auto &point) {
                       return std::abs(point.x) < 0.6f && std::abs(point.y) < 0.6f;
                     }),
      input.input_cloud_in_base.end());
  return input;
}

struct ScenarioSpec {
  std::string name;
  std::vector<FrameInput> frames;
  bool expect_rear_dropout = false;
  bool expect_low_dropout = false;
  bool expect_low_ceiling_impassable = false;
  bool expect_passable_corridor = false;
  bool expect_support_persistence = false;
};

std::vector<ScenarioSpec> BuildSyntheticScenarios() {
  return {
      {"rear_normal_open", {MakeRearNormalOpenScene(1)}, false, true, false, false, false},
      {"rear_dropout", {MakeFlatScene(1), MakeRearDropoutScene(100000001)}, true, false, false,
       false, true},
      {"stairs", {MakeStairScene(1)}, false, false, false, true, false},
      {"slope", {MakeSlopeScene(1)}, false, false, false, true, false},
      {"low_ceiling", {MakeLowCeilingScene(1)}, false, false, true, false, false},
      {"local_hole", {MakeFlatScene(1), MakeLocalHoleScene(100000001)}, false, false, false,
       false, true},
  };
}

struct ScenarioResult {
  std::string name;
  bool rear_dropout = false;
  int missing_sector_count = 0;
  int partial_sector_count = 0;
  int passable_count = 0;
  int impassable_count = 0;
  int unknown_count = 0;
  float max_rear_support_confidence = 0.0f;
  bool corridor_found = false;
  bool passed = false;
  std::vector<std::string> failures;
};

int CellIndex(const FrameOutput &output, float x, float y) {
  const int col = static_cast<int>(std::floor((x - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(std::floor((y - output.origin.y()) / output.resolution));
  if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
    return -1;
  }
  return row * output.cols + col;
}

bool HasPassableCorridor(const FrameOutput &output, float min_width_m, float min_length_m) {
  const int min_cols = std::max(1, static_cast<int>(std::ceil(min_width_m / output.resolution)));
  const int min_rows = std::max(1, static_cast<int>(std::ceil(min_length_m / output.resolution)));
  for (int row = 0; row + min_rows <= output.rows; ++row) {
    for (int col = 0; col + min_cols <= output.cols; ++col) {
      bool all_passable = true;
      for (int r = row; r < row + min_rows && all_passable; ++r) {
        for (int c = col; c < col + min_cols; ++c) {
          const int idx = r * output.cols + c;
          if (output.passability[idx] != static_cast<int8_t>(PassabilityState::kPassable)) {
            all_passable = false;
            break;
          }
        }
      }
      if (all_passable) {
        return true;
      }
    }
  }
  return false;
}

ScenarioResult RunScenario(const ScenarioSpec &spec) {
  ScenarioResult result;
  result.name = spec.name;
  passable_area::core::Processor processor(MakeConfig());
  FrameOutput output;
  for (const auto &frame : spec.frames) {
    output = processor.update(frame);
  }
  if (!output.valid) {
    result.failures.push_back("processor returned invalid output");
    return result;
  }

  result.rear_dropout = output.observability.rear_dropout;
  for (const auto &sector : output.observability.sectors) {
    if (sector.state == ObservabilityState::kMissingByDropout) {
      ++result.missing_sector_count;
    }
    if (sector.state == ObservabilityState::kPartiallyObserved) {
      ++result.partial_sector_count;
    }
  }
  for (const auto state : output.passability) {
    if (state == static_cast<int8_t>(PassabilityState::kPassable)) {
      ++result.passable_count;
    } else if (state == static_cast<int8_t>(PassabilityState::kImpassable)) {
      ++result.impassable_count;
    } else {
      ++result.unknown_count;
    }
  }
  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const int idx = row * output.cols + col;
      const float x = output.origin.x() + (static_cast<float>(col) + 0.5f) * output.resolution;
      if (x < -0.5f) {
        result.max_rear_support_confidence =
            std::max(result.max_rear_support_confidence, output.support_confidence[idx]);
      }
    }
  }
  result.corridor_found = HasPassableCorridor(output, 0.5f, 1.0f);

  if (spec.expect_rear_dropout && !result.rear_dropout) {
    result.failures.push_back("expected rear_dropout=true");
  }
  if (spec.expect_low_dropout && result.rear_dropout) {
    result.failures.push_back("expected rear_dropout=false");
  }
  if (spec.expect_low_ceiling_impassable && result.impassable_count == 0) {
    result.failures.push_back("expected impassable cells under low ceiling");
  }
  if (spec.expect_passable_corridor && !result.corridor_found) {
    result.failures.push_back("expected a passable corridor");
  }
  if (spec.expect_support_persistence && result.max_rear_support_confidence < 0.05f) {
    result.failures.push_back("expected persistent rear support confidence");
  }
  result.passed = result.failures.empty();
  return result;
}

void PrintScenarioResult(const ScenarioResult &result) {
  std::cout << "scenario=" << result.name << " rear_dropout=" << std::boolalpha
            << result.rear_dropout << " missing_sectors=" << result.missing_sector_count
            << " partial_sectors=" << result.partial_sector_count
            << " passable=" << result.passable_count << " impassable=" << result.impassable_count
            << " unknown=" << result.unknown_count << " max_rear_support_conf=" << std::fixed
            << std::setprecision(2) << result.max_rear_support_confidence
            << " corridor=" << result.corridor_found
            << " status=" << (result.passed ? "PASS" : "FAIL") << '\n';
  for (const auto &failure : result.failures) {
    std::cout << "  failure: " << failure << '\n';
  }
}

struct BagReplaySummary {
  int paired_frames = 0;
  int rear_dropout_frames = 0;
  int max_missing_sectors = 0;
  double avg_unknown_ratio = 0.0;
};

struct FalseObstacleReplayArgs {
  std::string bag_path;
  passable_area::tools::FalseObstacleAnalyzerConfig analyzer_config;
  int top_k = 10;
};

void PrintDetectionBox(const passable_area::tools::FalseObstacleDetectionBox &box) {
  std::cout << "detection_box="
            << "x[" << std::fixed << std::setprecision(2) << box.x_min << ", " << box.x_max
            << "] y[" << box.y_min << ", " << box.y_max << "]\n";
}

void PrintFalseObstacleFrame(const passable_area::tools::FalseObstacleFrameAnalysis &frame,
                             int rank) {
  std::cout << "frame_rank=" << rank << " stamp=" << frame.stamp
            << " in_box_obstacle_points=" << frame.in_box_obstacle_point_count
            << " hotspot_count=" << frame.hotspots.size() << " severity=" << std::fixed
            << std::setprecision(2) << frame.severity
            << " class=" << passable_area::tools::ToString(frame.classification)
            << " max_obstacle_evidence=" << frame.max_local_obstacle_evidence
            << " min_clearance=" << frame.min_local_clearance
            << " min_support_continuity=" << frame.min_local_support_continuity
            << " frame_partial=" << std::boolalpha << frame.frame_partial
            << " rear_dropout=" << frame.rear_dropout << '\n';

  for (size_t i = 0; i < frame.hotspots.size(); ++i) {
    const auto &hotspot = frame.hotspots[i];
    std::cout << "  hotspot_rank=" << (i + 1) << " x=" << std::fixed << std::setprecision(2)
              << hotspot.x << " y=" << hotspot.y
              << " obstacle_points=" << hotspot.obstacle_point_count
              << " severity=" << hotspot.severity
              << " obstacle_evidence=" << hotspot.obstacle_evidence
              << " clearance=" << hotspot.clearance
              << " support_continuity=" << hotspot.support_continuity << " observability=";
    if (hotspot.has_observability) {
      std::cout << passable_area::tools::ToString(hotspot.observability_state);
    } else {
      std::cout << "Unavailable";
    }
    std::cout << " class=" << passable_area::tools::ToString(hotspot.classification)
              << " explanation=\"" << hotspot.explanation << "\"\n";
  }
}

void PrintFalseObstacleSummary(const passable_area::tools::FalseObstacleBagSummary &summary,
                               const std::string &bag_path) {
  std::cout << "false_obstacle_summary bag=" << bag_path << '\n';
  PrintDetectionBox(summary.detection_box);
  const double candidate_ratio =
      summary.total_frames > 0
          ? static_cast<double>(summary.candidate_frames) / static_cast<double>(summary.total_frames)
          : 0.0;
  std::cout << "total_frames=" << summary.total_frames
            << " candidate_frames=" << summary.candidate_frames
            << " candidate_ratio=" << std::fixed << std::setprecision(3) << candidate_ratio
            << " longest_consecutive_run=" << summary.longest_consecutive_candidate_run << '\n';
  std::cout << "root_causes"
            << " ClearanceDriven="
            << summary.root_cause_counts[static_cast<int>(
                   passable_area::tools::FalseObstacleRootCause::kClearanceDriven)]
            << " ObstacleEvidenceDriven="
            << summary.root_cause_counts[static_cast<int>(
                   passable_area::tools::FalseObstacleRootCause::kObstacleEvidenceDriven)]
            << " ObstacleEvidencePlusLowContinuity="
            << summary.root_cause_counts[static_cast<int>(passable_area::tools::FalseObstacleRootCause::
                                                              kObstacleEvidencePlusLowContinuity)]
            << " ObservabilityInfluenced="
            << summary.root_cause_counts[static_cast<int>(
                   passable_area::tools::FalseObstacleRootCause::kObservabilityInfluenced)]
            << " UnknownOrMixed="
            << summary.root_cause_counts[static_cast<int>(
                   passable_area::tools::FalseObstacleRootCause::kUnknownOrMixed)]
            << '\n';
  for (size_t i = 0; i < summary.ranked_frames.size(); ++i) {
    PrintFalseObstacleFrame(summary.ranked_frames[i], static_cast<int>(i + 1));
  }
}

std::optional<float> ParseFloatFlagValue(const std::vector<std::string> &args, const std::string &flag) {
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) {
      return std::stof(args[i + 1]);
    }
  }
  return std::nullopt;
}

std::optional<int> ParseIntFlagValue(const std::vector<std::string> &args, const std::string &flag) {
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) {
      return std::stoi(args[i + 1]);
    }
  }
  return std::nullopt;
}

bool HasFlag(const std::vector<std::string> &args, const std::string &flag) {
  return std::find(args.begin(), args.end(), flag) != args.end();
}

std::optional<std::string> ParseStringFlagValue(const std::vector<std::string> &args,
                                                const std::string &flag) {
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) {
      return args[i + 1];
    }
  }
  return std::nullopt;
}

std::optional<FalseObstacleReplayArgs> ParseFalseObstacleReplayArgs(
    const std::vector<std::string> &args) {
  const auto bag_path = ParseStringFlagValue(args, "--bag");
  const auto range_x_min = ParseFloatFlagValue(args, "--range-x-min");
  const auto range_x_max = ParseFloatFlagValue(args, "--range-x-max");
  const auto range_y_min = ParseFloatFlagValue(args, "--range-y-min");
  const auto range_y_max = ParseFloatFlagValue(args, "--range-y-max");
  if (!bag_path || !range_x_min || !range_x_max || !range_y_min || !range_y_max) {
    std::cerr << "false-obstacle analysis requires --bag, --range-x-min, --range-x-max, "
                 "--range-y-min, and --range-y-max\n";
    return std::nullopt;
  }

  FalseObstacleReplayArgs replay_args;
  replay_args.bag_path = *bag_path;
  replay_args.analyzer_config.detection_box = passable_area::tools::FalseObstacleDetectionBox{
      *range_x_min, *range_x_max, *range_y_min, *range_y_max};
  if (const auto top_k = ParseIntFlagValue(args, "--top-k")) {
    replay_args.top_k = *top_k;
  }
  if (!(replay_args.analyzer_config.detection_box.x_min <
        replay_args.analyzer_config.detection_box.x_max)) {
    std::cerr << "invalid detection box: require --range-x-min < --range-x-max\n";
    return std::nullopt;
  }
  if (!(replay_args.analyzer_config.detection_box.y_min <
        replay_args.analyzer_config.detection_box.y_max)) {
    std::cerr << "invalid detection box: require --range-y-min < --range-y-max\n";
    return std::nullopt;
  }
  if (replay_args.top_k <= 0) {
    std::cerr << "invalid --top-k: require top_k > 0\n";
    return std::nullopt;
  }
  return replay_args;
}

std::optional<BagReplaySummary> RunBagReplay(const std::string &bag_path) {
  if (!std::filesystem::exists(bag_path)) {
    std::cerr << "bag path does not exist: " << bag_path << '\n';
    return std::nullopt;
  }
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  const auto config = MakeConfig();
  passable_area::core::Processor processor(config);
  BagReplaySummary summary;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == "/LOC_BODY_POINTS") {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      clouds.emplace(rclcpp::Time(cloud_msg.header.stamp).nanoseconds(), std::move(cloud_msg));
    } else if (bag_msg->topic_name == "/ODOM") {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(), std::move(odom_msg));
    }
  }

  double unknown_ratio_sum = 0.0;
  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) || !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_odom = pose;
    input.input_cloud_in_base = std::move(cloud);
    const auto output = processor.update(input);
    if (!output.valid) {
      continue;
    }
    ++summary.paired_frames;
    if (output.observability.rear_dropout) {
      ++summary.rear_dropout_frames;
    }
    int missing = 0;
    for (const auto &sector : output.observability.sectors) {
      if (sector.state == ObservabilityState::kMissingByDropout) {
        ++missing;
      }
    }
    summary.max_missing_sectors = std::max(summary.max_missing_sectors, missing);
    const double unknown_ratio =
        static_cast<double>(std::count(output.passability.begin(), output.passability.end(),
                                       static_cast<int8_t>(PassabilityState::kUnknown))) /
        std::max<size_t>(output.passability.size(), 1U);
    unknown_ratio_sum += unknown_ratio;
  }

  if (summary.paired_frames > 0) {
    summary.avg_unknown_ratio = unknown_ratio_sum / static_cast<double>(summary.paired_frames);
  }
  return summary;
}

std::optional<passable_area::tools::FalseObstacleBagSummary> RunFalseObstacleReplay(
    const FalseObstacleReplayArgs &args) {
  if (!std::filesystem::exists(args.bag_path)) {
    std::cerr << "bag path does not exist: " << args.bag_path << '\n';
    return std::nullopt;
  }

  std::cout << "false_obstacle_analysis bag=" << args.bag_path << '\n';
  PrintDetectionBox(args.analyzer_config.detection_box);
  std::cout << "top_k=" << args.top_k << '\n';

  rosbag2_cpp::Reader reader;
  reader.open(args.bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == "/LOC_BODY_POINTS") {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      clouds.emplace(rclcpp::Time(cloud_msg.header.stamp).nanoseconds(), std::move(cloud_msg));
    } else if (bag_msg->topic_name == "/ODOM") {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(), std::move(odom_msg));
    }
  }

  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  const auto config = MakeConfig();
  passable_area::core::Processor processor(config);
  passable_area::tools::FalseObstacleAnalyzer analyzer(config, args.analyzer_config);

  int total_frames = 0;
  int current_candidate_run = 0;
  int longest_candidate_run = 0;
  std::vector<passable_area::tools::FalseObstacleFrameAnalysis> candidate_frames;

  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) || !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_odom = pose;
    input.input_cloud_in_base = std::move(cloud);
    const auto output = processor.update(input);
    if (!output.valid) {
      continue;
    }

    ++total_frames;
    const auto analysis = analyzer.analyzeFrame(output);
    if (analysis) {
      candidate_frames.push_back(*analysis);
      ++current_candidate_run;
      longest_candidate_run = std::max(longest_candidate_run, current_candidate_run);
    } else {
      current_candidate_run = 0;
    }
  }

  return analyzer.buildSummary(total_frames, longest_candidate_run, std::move(candidate_frames),
                               args.top_k);
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::vector<std::string> args(argv + 1, argv + argc);

  if (HasFlag(args, "--analyze-false-obstacles")) {
    const auto replay_args = ParseFalseObstacleReplayArgs(args);
    if (!replay_args) {
      rclcpp::shutdown();
      return 1;
    }
    const auto summary = RunFalseObstacleReplay(*replay_args);
    if (!summary) {
      rclcpp::shutdown();
      return 1;
    }
    PrintFalseObstacleSummary(*summary, replay_args->bag_path);
    rclcpp::shutdown();
    return 0;
  }

  if (args.size() >= 2 && args[0] == "--bag") {
    const auto summary = RunBagReplay(args[1]);
    if (!summary) {
      rclcpp::shutdown();
      return 1;
    }
    std::cout << "bag=" << args[1] << " paired_frames=" << summary->paired_frames
              << " rear_dropout_frames=" << summary->rear_dropout_frames
              << " max_missing_sectors=" << summary->max_missing_sectors
              << " avg_unknown_ratio=" << std::fixed << std::setprecision(3)
              << summary->avg_unknown_ratio << '\n';
    rclcpp::shutdown();
    return 0;
  }

  bool all_passed = true;
  for (const auto &scenario : BuildSyntheticScenarios()) {
    const auto result = RunScenario(scenario);
    PrintScenarioResult(result);
    all_passed = all_passed && result.passed;
  }
  rclcpp::shutdown();
  return all_passed ? 0 : 1;
}
