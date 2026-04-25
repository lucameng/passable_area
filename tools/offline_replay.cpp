#include "passable_area/interfaces/ros/converters/odom_converter.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/interfaces/ros/ros_param_loader.hpp"
#include "passable_area/passable_area.hpp"
#include "passable_area/core/types/obstacle_types.hpp"
#include "passable_area/tools/false_obstacle_analyzer.hpp"
#include "passable_area/tools/miss_obstacle_analyzer.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#if __has_include(<rosbag2_storage/storage_options.hpp>)
#include <rosbag2_storage/storage_options.hpp>
using BagStorageOptions = rosbag2_storage::StorageOptions;
#else
#include <rosbag2_cpp/storage_options.hpp>
using BagStorageOptions = rosbag2_cpp::StorageOptions;
#endif
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using passable_area::core::FrameInput;
using passable_area::core::FrameOutput;
using passable_area::core::ObservabilityState;
using passable_area::core::PassabilityState;
using passable_area::core::ProcessedFrame;

const char *ToBlockReasonString(passable_area::core::BlockReason reason);
const char *ToObstaclePointPublishStatusString(
    passable_area::core::ObstaclePointPublishStatus status);

bool IsLowClearanceBridgeEligible(const passable_area::core::Config &config,
                                  uint8_t block_reason,
                                  float clearance,
                                  float overhead_evidence) {
  return block_reason ==
         static_cast<uint8_t>(
             passable_area::core::BlockReason::kLowClearance) &&
     std::isfinite(clearance) && clearance > config.geometry.max_step_up &&
         overhead_evidence >= config.obstacle_points_min_evidence;
}

std::string PublishPathFromStatus(
    passable_area::core::ObstaclePointPublishStatus publish_status) {
  switch (publish_status) {
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByProtrusion:
    return "ProtrusionEvidence";
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByDenseProtrusion:
    return "DenseProtrusionSource";
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByGeometryFailure:
    return "GeometryFailure";
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByOverhead:
    return "LowClearanceBridge";
  case passable_area::core::ObstaclePointPublishStatus::kGatedByEvidence:
    return "GatedByEvidence";
  case passable_area::core::ObstaclePointPublishStatus::kGatedByHeight:
    return "GatedByHeight";
  case passable_area::core::ObstaclePointPublishStatus::kBlockedButNoSamples:
    return "BlockedButNoSamples";
  case passable_area::core::ObstaclePointPublishStatus::kNotApplicable:
    return "None";
  }
  return "None";
}

std::string DefaultParamsFile();
std::vector<std::string> DefaultParamsFiles();
std::optional<passable_area::interfaces::ros::RosNodeParams>
LoadNodeParamsFromParamsFiles(const std::vector<std::string> &params_files);

std::unique_ptr<rosbag2_cpp::Reader>
OpenBagReader(const std::string &bag_path) {
  auto reader = std::make_unique<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  BagStorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};
  reader->open(storage_options, converter_options);
  return reader;
}

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
  input.base_pose_in_map.position = Eigen::Vector3f::Zero();
  input.base_pose_in_map.orientation = Eigen::Quaternionf::Identity();
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
      std::remove_if(
          input.input_cloud_in_base.begin(), input.input_cloud_in_base.end(),
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
      {"rear_normal_open",
       {MakeRearNormalOpenScene(1)},
       false,
       true,
       false,
       false,
       false},
      {"rear_dropout",
       {MakeFlatScene(1), MakeRearDropoutScene(100000001)},
       true,
       false,
       false,
       false,
       true},
      {"stairs", {MakeStairScene(1)}, false, false, false, true, false},
      {"slope", {MakeSlopeScene(1)}, false, false, false, true, false},
      {"low_ceiling",
       {MakeLowCeilingScene(1)},
       false,
       false,
       true,
       false,
       false},
      {"local_hole",
       {MakeFlatScene(1), MakeLocalHoleScene(100000001)},
       false,
       false,
       false,
       false,
       true},
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
  const int col =
      static_cast<int>(std::floor((x - output.origin.x()) / output.resolution));
  const int row =
      static_cast<int>(std::floor((y - output.origin.y()) / output.resolution));
  if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
    return -1;
  }
  return row * output.cols + col;
}

bool HasPassableCorridor(const FrameOutput &output, float min_width_m,
                         float min_length_m) {
  const int min_cols =
      std::max(1, static_cast<int>(std::ceil(min_width_m / output.resolution)));
  const int min_rows = std::max(
      1, static_cast<int>(std::ceil(min_length_m / output.resolution)));
  for (int row = 0; row + min_rows <= output.rows; ++row) {
    for (int col = 0; col + min_cols <= output.cols; ++col) {
      bool all_passable = true;
      for (int r = row; r < row + min_rows && all_passable; ++r) {
        for (int c = col; c < col + min_cols; ++c) {
          const int idx = r * output.cols + c;
          if (output.passability[idx] !=
              static_cast<int8_t>(PassabilityState::kPassable)) {
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
      const float x = output.origin.x() +
                      (static_cast<float>(col) + 0.5f) * output.resolution;
      if (x < -0.5f) {
        result.max_rear_support_confidence = std::max(
            result.max_rear_support_confidence, output.support_confidence[idx]);
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
  if (spec.expect_support_persistence &&
      result.max_rear_support_confidence < 0.05f) {
    result.failures.push_back("expected persistent rear support confidence");
  }
  result.passed = result.failures.empty();
  return result;
}

void PrintScenarioResult(const ScenarioResult &result) {
  std::cout << "scenario=" << result.name << " rear_dropout=" << std::boolalpha
            << result.rear_dropout
            << " missing_sectors=" << result.missing_sector_count
            << " partial_sectors=" << result.partial_sector_count
            << " passable=" << result.passable_count
            << " impassable=" << result.impassable_count
            << " unknown=" << result.unknown_count
            << " max_rear_support_conf=" << std::fixed << std::setprecision(2)
            << result.max_rear_support_confidence
            << " corridor=" << result.corridor_found
            << " status=" << (result.passed ? "PASS" : "FAIL") << '\n';
  for (const auto &failure : result.failures) {
    std::cout << "  failure: " << failure << '\n';
  }
}

struct BagReplaySummary {
  int cloud_total_frames = 0;
  int odom_total_frames = 0;
  int paired_frames = 0;
  int cloud_unpaired_frames = 0;
  int odom_unpaired_frames = 0;
  int invalid_frames = 0;
  int rear_dropout_frames = 0;
  int max_missing_sectors = 0;
  double avg_unknown_ratio = 0.0;
  double max_unknown_ratio = 0.0;
  double avg_input_point_count = 0.0;
  double max_input_point_count = 0.0;
  double avg_processing_ms = 0.0;
  double max_processing_ms = 0.0;
  double p95_processing_ms = 0.0;
};

struct FalseObstacleReplayArgs {
  std::string bag_path;
  std::string params_file;
  passable_area::tools::FalseObstacleAnalyzerConfig analyzer_config;
  float start_offset_sec = 0.0f;
  float time_window_sec = 0.2f;
  bool use_analysis_window = false;
  int top_k = 10;
  bool use_color = true;
};

struct MissObstacleReplayArgs {
  std::string bag_path;
  std::string params_file;
  passable_area::tools::MissObstacleAnalyzerConfig analyzer_config;
  float start_offset_sec = 0.0f;
  float time_window_sec = 0.2f;
  bool use_color = true;
};

struct RoiInspectArgs {
  std::string bag_path;
  std::string params_file;
  float start_offset_sec = 0.0f;
  float time_window_sec = 0.2f;
  float roi_x_min = 0.0f;
  float roi_x_max = 0.0f;
  float roi_y_min = 0.0f;
  float roi_y_max = 0.0f;
};

struct TerminalStyle {
  bool use_color = true;
};

std::string Colorize(const std::string &text, const char *ansi_code,
                     const TerminalStyle &style) {
  if (!style.use_color) {
    return text;
  }
  return std::string(ansi_code) + text + "\033[0m";
}

std::string RootCauseColor(passable_area::tools::FalseObstacleRootCause cause) {
  switch (cause) {
  case passable_area::tools::FalseObstacleRootCause::kClearanceDriven:
    return "\033[33m";
  case passable_area::tools::FalseObstacleRootCause::kObstacleEvidenceDriven:
    return "\033[31m";
  case passable_area::tools::FalseObstacleRootCause::
      kObstacleEvidencePlusLowContinuity:
    return "\033[35m";
  case passable_area::tools::FalseObstacleRootCause::kObservabilityInfluenced:
    return "\033[34m";
  case passable_area::tools::FalseObstacleRootCause::kUnknownOrMixed:
    return "\033[37m";
  }
  return "\033[37m";
}

std::string ObservabilityColor(passable_area::core::ObservabilityState state) {
  switch (state) {
  case passable_area::core::ObservabilityState::kObserved:
    return "\033[32m";
  case passable_area::core::ObservabilityState::kMissingByDropout:
    return "\033[31m";
  }
  return "\033[37m";
}

std::string BoolBadge(bool value, const TerminalStyle &style) {
  return Colorize(value ? "true" : "false", value ? "\033[32m" : "\033[90m",
                  style);
}

std::string FormatFloat(double value, int precision = 2) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

double Percentile(std::vector<double> samples, double p) {
  if (samples.empty()) {
    return 0.0;
  }
  std::sort(samples.begin(), samples.end());
  const size_t idx =
      std::min(samples.size() - 1,
               static_cast<size_t>(std::floor(p * (samples.size() - 1))));
  return samples[idx];
}

std::string
FormatDetectionBox(const passable_area::tools::FalseObstacleDetectionBox &box) {
  std::ostringstream oss;
  oss << "x[" << std::fixed << std::setprecision(2) << box.x_min << ", "
      << box.x_max << "] y[" << box.y_min << ", " << box.y_max << "]";
  return oss.str();
}

std::string
FormatDetectionBox(const passable_area::tools::MissObstacleDetectionBox &box) {
  std::ostringstream oss;
  oss << "x[" << std::fixed << std::setprecision(2) << box.x_min << ", "
      << box.x_max << "] y[" << box.y_min << ", " << box.y_max << "]";
  return oss.str();
}

constexpr size_t kCardColumns = 72;
constexpr size_t kSectionHeaderColumns = kCardColumns + 2;

std::string RepeatGlyph(const std::string &glyph, size_t count) {
  std::string out;
  out.reserve(glyph.size() * count);
  for (size_t i = 0; i < count; ++i) {
    out += glyph;
  }
  return out;
}

void PrintBanner(const std::string &title, const TerminalStyle &style) {
  std::cout << Colorize("╔" + RepeatGlyph("═", kCardColumns) + "╗", "\033[36m",
                        style)
            << '\n';
  std::cout << Colorize("║ " + title, "\033[1;36m", style) << '\n';
  std::cout << Colorize("╚" + RepeatGlyph("═", kCardColumns) + "╝", "\033[36m",
                        style)
            << '\n';
}

void PrintSectionHeader(const std::string &title, const TerminalStyle &style) {
  std::string line = "╭─ " + title + " ";
  const size_t visible_prefix_columns = 3 + title.size() + 1;
  if (visible_prefix_columns < kSectionHeaderColumns) {
    const size_t fill_count = kSectionHeaderColumns - visible_prefix_columns;
    for (size_t i = 0; i < fill_count; ++i) {
      line += "─";
    }
  }
  std::cout << '\n' << Colorize(line, "\033[36m", style) << '\n';
}

void PrintKeyValueLine(const std::string &label, const std::string &value) {
  std::cout << "  " << label << ": " << value << '\n';
}

void PrintDetectionBox(
    const passable_area::tools::FalseObstacleDetectionBox &box,
    const TerminalStyle &style) {
  PrintKeyValueLine("detection_box",
                    Colorize(FormatDetectionBox(box), "\033[36m", style));
}

void PrintDetectionBox(
    const passable_area::tools::MissObstacleDetectionBox &box,
    const TerminalStyle &style) {
  PrintKeyValueLine("roi_box",
                    Colorize(FormatDetectionBox(box), "\033[36m", style));
}

void PrintFalseObstacleFrame(
    const passable_area::tools::FalseObstacleFrameAnalysis &frame, int rank,
    const TerminalStyle &style) {
  const auto class_label =
      Colorize(passable_area::tools::ToString(frame.classification),
               RootCauseColor(frame.classification).c_str(), style);
  std::cout << '\n'
            << Colorize("╔" + RepeatGlyph("═", kCardColumns) + "╗", "\033[36m",
                        style)
            << '\n';
  std::cout << Colorize("║ frame " + std::to_string(rank), "\033[1;36m", style)
            << "  " << class_label << "  severity="
            << Colorize(FormatFloat(frame.severity), "\033[1;33m", style)
            << '\n';
  std::cout << Colorize("╟" + RepeatGlyph("─", kCardColumns) + "╢", "\033[36m",
                        style)
            << '\n';
  PrintKeyValueLine("stamp", std::to_string(frame.stamp));
  PrintKeyValueLine("start_offset",
                    FormatFloat(frame.start_offset_sec, 3) + " s");
  PrintKeyValueLine("in_box_obstacle_points",
                    std::to_string(frame.in_box_obstacle_point_count));
  PrintKeyValueLine("hotspot_count", std::to_string(frame.hotspots.size()));
  PrintKeyValueLine("frame_partial", BoolBadge(frame.frame_partial, style));
  PrintKeyValueLine("rear_dropout", BoolBadge(frame.rear_dropout, style));
  PrintKeyValueLine("max_obstacle_evidence",
                    FormatFloat(frame.max_local_obstacle_evidence));
  PrintKeyValueLine("min_clearance", FormatFloat(frame.min_local_clearance));
  PrintKeyValueLine("min_support_continuity",
                    FormatFloat(frame.min_local_support_continuity));

  for (size_t i = 0; i < frame.hotspots.size(); ++i) {
    const auto &hotspot = frame.hotspots[i];
    const std::string hotspot_class =
        Colorize(passable_area::tools::ToString(hotspot.classification),
                 RootCauseColor(hotspot.classification).c_str(), style);
    const std::string observability =
        hotspot.has_observability
            ? Colorize(
                  passable_area::tools::ToString(hotspot.observability_state),
                  ObservabilityColor(hotspot.observability_state).c_str(),
                  style)
            : "Unavailable";
    std::cout << "  "
              << Colorize("• hotspot " + std::to_string(i + 1), "\033[1;35m",
                          style)
              << '\n';
    std::cout << "    pos: (" << FormatFloat(hotspot.x) << ", "
              << FormatFloat(hotspot.y) << ")  z: ["
              << FormatFloat(hotspot.min_z) << ", "
              << FormatFloat(hotspot.max_z)
              << "]  obstacle_points: " << hotspot.obstacle_point_count
              << "  severity: "
              << Colorize(FormatFloat(hotspot.severity), "\033[1;33m", style)
              << '\n';
    std::cout << "    source_cell: " << hotspot.source_cell
              << "  center_cell: " << hotspot.center_cell
              << "  source_matches_center: "
              << (hotspot.source_matches_center ? "true" : "false") << '\n';
    std::cout << "    class: " << hotspot_class
              << "  observability: " << observability << '\n';
    std::cout << "    why: "
              << Colorize(hotspot.explanation, "\033[1;37m", style) << '\n';
    std::cout << "    obstacle_evidence: "
              << FormatFloat(hotspot.obstacle_evidence)
              << "  protrusion_evidence: "
              << FormatFloat(hotspot.protrusion_evidence)
              << "  overhead_evidence: "
              << FormatFloat(hotspot.overhead_evidence)
              << "  clearance: " << FormatFloat(hotspot.clearance)
              << "  support_continuity: "
              << FormatFloat(hotspot.support_continuity) << '\n';
    std::cout << "    block_reason: "
              << ToBlockReasonString(
                     static_cast<passable_area::core::BlockReason>(
                         hotspot.block_reason))
              << '\n';
    std::cout << "    source_obstacle_evidence: "
              << FormatFloat(hotspot.source_obstacle_evidence)
              << "  source_protrusion_evidence: "
              << FormatFloat(hotspot.source_protrusion_evidence)
              << "  source_overhead_evidence: "
              << FormatFloat(hotspot.source_overhead_evidence) << '\n';
    std::cout << "    source_clearance: "
              << FormatFloat(hotspot.source_clearance)
              << "  source_support_continuity: "
              << FormatFloat(hotspot.source_support_continuity)
              << "  source_support_height: "
              << FormatFloat(hotspot.source_support_height)
              << "  source_overhead_height: "
              << FormatFloat(hotspot.source_overhead_height) << '\n';
    std::cout << "    source_block_reason: "
              << ToBlockReasonString(
                     static_cast<passable_area::core::BlockReason>(
                         hotspot.source_block_reason))
              << "  source_publish_status: "
              << ToObstaclePointPublishStatusString(
                     static_cast<
                         passable_area::core::ObstaclePointPublishStatus>(
                         hotspot.source_obstacle_point_publish_status))
              << "  source_publish_path: " << hotspot.source_publish_path
              << "  low_clearance_bridge_hit: "
              << (hotspot.source_low_clearance_bridge_eligible ? "true"
                                                               : "false")
              << '\n';
    std::cout << "    raw_sample_z: [" << FormatFloat(hotspot.raw_sample_min_z)
              << ", " << FormatFloat(hotspot.raw_sample_max_z)
              << "]  raw_sample_count: "
              << std::to_string(hotspot.raw_sample_count)
              << "  filtered_sample_z: ["
              << FormatFloat(hotspot.filtered_sample_min_z) << ", "
              << FormatFloat(hotspot.filtered_sample_max_z)
              << "]  filtered_sample_count: "
              << std::to_string(hotspot.filtered_sample_count) << '\n';
    std::cout
        << "    obstacle_suspicious: "
        << (hotspot.obstacle_suspicious ? "true" : "false")
        << "  obstacle_candidate_cell: "
        << (hotspot.obstacle_candidate_cell ? "true" : "false")
              << '\n';
  }
  std::cout << Colorize("╚" + RepeatGlyph("═", kCardColumns) + "╝", "\033[36m",
                        style)
            << '\n';
}

void PrintFalseObstacleSummary(
    const passable_area::tools::FalseObstacleBagSummary &summary,
    const std::string &bag_path, const std::string &params_file,
    const FalseObstacleReplayArgs &args, const TerminalStyle &style) {
  PrintBanner("False Obstacle Offline Analysis", style);
  PrintSectionHeader("Run", style);
  PrintKeyValueLine("bag", bag_path);
  PrintKeyValueLine("params_file", params_file);
  PrintDetectionBox(summary.detection_box, style);
  if (args.use_analysis_window) {
    PrintKeyValueLine("start_offset_sec",
                      FormatFloat(args.start_offset_sec, 3));
    PrintKeyValueLine("time_window_sec", FormatFloat(args.time_window_sec, 3));
  }
  PrintKeyValueLine("top_k", std::to_string(args.top_k));

  const double candidate_ratio =
      summary.total_frames > 0 ? static_cast<double>(summary.candidate_frames) /
                                     static_cast<double>(summary.total_frames)
                               : 0.0;
  PrintSectionHeader("Summary", style);
  PrintKeyValueLine("total_frames", std::to_string(summary.total_frames));
  PrintKeyValueLine("rear_dropout_frames",
                    std::to_string(summary.rear_dropout_frames));
  PrintKeyValueLine(
      "suspicious_frames",
      Colorize(std::to_string(summary.candidate_frames),
               summary.candidate_frames > 0 ? "\033[1;31m" : "\033[1;32m",
               style));
  PrintKeyValueLine("suspicious_ratio", FormatFloat(candidate_ratio, 3));
  PrintKeyValueLine("longest_consecutive_run",
                    std::to_string(summary.longest_consecutive_candidate_run));

  PrintSectionHeader("Root Causes", style);
  const auto print_root_cause =
      [&](passable_area::tools::FalseObstacleRootCause cause) {
        const std::string label = passable_area::tools::ToString(cause);
        const std::string value =
            std::to_string(summary.root_cause_counts[static_cast<int>(cause)]);
        PrintKeyValueLine(Colorize(label, RootCauseColor(cause).c_str(), style),
                          value);
      };
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kClearanceDriven);
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kObstacleEvidenceDriven);
  print_root_cause(passable_area::tools::FalseObstacleRootCause::
                       kObstacleEvidencePlusLowContinuity);
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kObservabilityInfluenced);
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kUnknownOrMixed);
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kProtrusionEvidenceDriven);
  print_root_cause(
      passable_area::tools::FalseObstacleRootCause::kOverheadEvidenceDriven);

  PrintSectionHeader("Ranked Frames", style);
  if (summary.ranked_frames.empty()) {
    std::cout << Colorize("╭" + RepeatGlyph("─", kCardColumns) + "╮",
                          "\033[32m", style)
              << '\n';
    std::cout << Colorize("│ No Candidate Frames", "\033[1;32m", style) << '\n';
    std::cout << "  no obstacle_points were found inside the detection box\n";
    std::cout << Colorize("╰" + RepeatGlyph("─", kCardColumns) + "╯",
                          "\033[32m", style)
              << '\n';
    return;
  }
  for (size_t i = 0; i < summary.ranked_frames.size(); ++i) {
    PrintFalseObstacleFrame(summary.ranked_frames[i], static_cast<int>(i + 1),
                            style);
  }
}

std::string
MissRootCauseColor(passable_area::tools::MissObstacleRootCause cause) {
  switch (cause) {
  case passable_area::tools::MissObstacleRootCause::kNoSamplesInRoi:
    return "\033[34m";
  case passable_area::tools::MissObstacleRootCause::
      kNoFrontendObstacleSuspicion:
    return "\033[36m";
  case passable_area::tools::MissObstacleRootCause::kObstacleEvidenceTooLow:
    return "\033[31m";
  case passable_area::tools::MissObstacleRootCause::kOutputHeightGateNotMet:
    return "\033[31m";
  case passable_area::tools::MissObstacleRootCause::
      kNoObstacleSourceSamplesInRoi:
    return "\033[33m";
  case passable_area::tools::MissObstacleRootCause::kUnknownOrMixed:
    return "\033[37m";
  case passable_area::tools::MissObstacleRootCause::kReasonerNotBlocked:
    return "\033[35m";
  }
  return "\033[37m";
}

const char *ToBlockReasonString(passable_area::core::BlockReason reason) {
  switch (reason) {
  case passable_area::core::BlockReason::kNone:
    return "None";
  case passable_area::core::BlockReason::kProtrusion:
    return "Protrusion";
  case passable_area::core::BlockReason::kLowClearance:
    return "LowClearance";
  case passable_area::core::BlockReason::kGeometryFailure:
    return "GeometryFailure";
  case passable_area::core::BlockReason::kMixed:
    return "Mixed";
  }
  return "Unknown";
}

const char *ToObstaclePointPublishStatusString(
    passable_area::core::ObstaclePointPublishStatus status) {
  switch (status) {
  case passable_area::core::ObstaclePointPublishStatus::kNotApplicable:
    return "NotApplicable";
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByProtrusion:
    return "PublishedByProtrusion";
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByDenseProtrusion:
    return "PublishedByDenseProtrusion";
  case passable_area::core::ObstaclePointPublishStatus::
      kPublishedByGeometryFailure:
    return "PublishedByGeometryFailure";
  case passable_area::core::ObstaclePointPublishStatus::kPublishedByOverhead:
    return "PublishedByOverhead";
  case passable_area::core::ObstaclePointPublishStatus::kGatedByEvidence:
    return "GatedByEvidence";
  case passable_area::core::ObstaclePointPublishStatus::kGatedByHeight:
    return "GatedByHeight";
  case passable_area::core::ObstaclePointPublishStatus::kBlockedButNoSamples:
    return "BlockedButNoSamples";
  }
  return "Unknown";
}

void PrintMissObstacleFrame(
    const passable_area::tools::MissObstacleFrameAnalysis &frame, int rank,
    const TerminalStyle &style) {
  const auto class_label =
      Colorize(passable_area::tools::ToString(frame.classification),
               MissRootCauseColor(frame.classification).c_str(), style);
  std::cout << '\n'
            << Colorize("╔" + RepeatGlyph("═", kCardColumns) + "╗", "\033[36m",
                        style)
            << '\n';
  std::cout << Colorize("║ frame " + std::to_string(rank), "\033[1;36m", style)
            << "  " << class_label << "  severity="
            << Colorize(FormatFloat(frame.severity), "\033[1;33m", style)
            << '\n';
  std::cout << Colorize("╟" + RepeatGlyph("─", kCardColumns) + "╢", "\033[36m",
                        style)
            << '\n';
  PrintKeyValueLine("stamp", std::to_string(frame.stamp));
  PrintKeyValueLine("start_offset",
                    FormatFloat(frame.start_offset_sec, 3) + " s");
  PrintKeyValueLine("roi_sample_count", std::to_string(frame.roi_sample_count));
  PrintKeyValueLine("roi_cells_with_any_samples",
                    std::to_string(frame.roi_cells_with_any_samples));
  PrintKeyValueLine("roi_obstacle_point_count",
                    std::to_string(frame.roi_obstacle_point_count));
  PrintKeyValueLine("obstacle_suspicious_cells",
                    std::to_string(frame.obstacle_suspicious_cell_count));
  PrintKeyValueLine("obstacle_candidate_cells",
                    std::to_string(frame.obstacle_candidate_cell_count));
  PrintKeyValueLine("max_obstacle_evidence",
                    FormatFloat(frame.max_obstacle_evidence));
  PrintKeyValueLine("max_support_confidence",
                    FormatFloat(frame.max_support_confidence));
  PrintKeyValueLine("min_clearance", FormatFloat(frame.min_clearance));
  PrintKeyValueLine("min_support_continuity",
                    FormatFloat(frame.min_support_continuity));
  PrintKeyValueLine("why", Colorize(frame.explanation, "\033[1;37m", style));
  for (const auto &evidence : frame.evidence_lines) {
    std::cout << "  " << Colorize("• evidence", "\033[1;35m", style) << ": "
              << evidence << '\n';
  }
  for (size_t i = 0; i < frame.representative_cells.size(); ++i) {
    const auto &cell = frame.representative_cells[i];
    std::cout << "  "
              << Colorize("• cell " + std::to_string(i + 1), "\033[1;35m",
                          style)
              << '\n';
    std::cout << "    pos: (" << FormatFloat(cell.x) << ", "
              << FormatFloat(cell.y) << ")  sample_relative_z: ["
              << FormatFloat(cell.min_sample_relative_z) << ", "
              << FormatFloat(cell.max_sample_relative_z)
              << "]  sample_count: " << cell.sample_count << '\n';
    std::cout << "    support_height: " << FormatFloat(cell.support_height)
              << "  support_ref: " << FormatFloat(cell.support_ref)
              << "  overhead_height: " << FormatFloat(cell.overhead_height)
              << '\n';
    std::cout << "    obstacle_evidence: "
              << FormatFloat(cell.obstacle_evidence)
              << "  protrusion_evidence: "
              << FormatFloat(cell.protrusion_evidence)
              << "  overhead_evidence: " << FormatFloat(cell.overhead_evidence)
              << '\n';
    std::cout << "    block_reason: "
              << ToBlockReasonString(
                     static_cast<passable_area::core::BlockReason>(
                         cell.block_reason))
              << "  obstacle_point_publish_status: "
              << ToObstaclePointPublishStatusString(
                     static_cast<
                         passable_area::core::ObstaclePointPublishStatus>(
                         cell.obstacle_point_publish_status))
              << '\n';
    std::cout << "    support_confidence: "
              << FormatFloat(cell.support_confidence)
              << "  support_continuity: "
              << FormatFloat(cell.support_continuity) << '\n';
    std::cout << "    max_sample_z_minus_support_ref: "
              << FormatFloat(cell.max_sample_z_minus_support_ref) << '\n';
    std::cout << "    raw_sample_z: [" << FormatFloat(cell.raw_sample_min_z)
              << ", " << FormatFloat(cell.raw_sample_max_z)
              << "]  raw_sample_count: "
              << std::to_string(cell.raw_sample_count)
              << "  filtered_sample_z: ["
              << FormatFloat(cell.filtered_sample_min_z) << ", "
              << FormatFloat(cell.filtered_sample_max_z)
              << "]  filtered_sample_count: "
              << std::to_string(cell.filtered_sample_count) << '\n';
    std::cout << "    obstacle_suspicious: "
              << (cell.obstacle_suspicious ? "true" : "false")
              << "  obstacle_candidate_cell: "
              << (cell.obstacle_candidate_cell ? "true" : "false")
              << '\n';
    std::cout << "    why: " << Colorize(cell.explanation, "\033[1;37m", style)
              << '\n';
  }
  std::cout << Colorize("╚" + RepeatGlyph("═", kCardColumns) + "╝", "\033[36m",
                        style)
            << '\n';
}

void PrintMissObstacleSummary(
    const passable_area::tools::MissObstacleBagSummary &summary,
    const MissObstacleReplayArgs &args, const TerminalStyle &style) {
  PrintBanner("Miss Obstacle Offline Analysis", style);
  PrintSectionHeader("Run", style);
  PrintKeyValueLine("bag", args.bag_path);
  PrintKeyValueLine("params_file", args.params_file);
  PrintDetectionBox(summary.detection_box, style);
  PrintKeyValueLine("start_offset",
                    FormatFloat(args.start_offset_sec, 3) + " s");
  PrintKeyValueLine("time_window", FormatFloat(args.time_window_sec, 3) + " s");
  PrintKeyValueLine(
      "top_k_cells",
      std::to_string(args.analyzer_config.representative_cell_limit));

  const double candidate_ratio =
      summary.total_frames > 0 ? static_cast<double>(summary.missed_frames) /
                                     static_cast<double>(summary.total_frames)
                               : 0.0;
  PrintSectionHeader("Summary", style);
  PrintKeyValueLine("total_frames", std::to_string(summary.total_frames));
  PrintKeyValueLine(
      "missed_frames",
      Colorize(std::to_string(summary.missed_frames),
               summary.missed_frames > 0 ? "\033[1;31m" : "\033[1;32m", style));
  PrintKeyValueLine("missed_ratio", FormatFloat(candidate_ratio, 3));

  PrintSectionHeader("Root Causes", style);
  const auto print_root_cause =
      [&](passable_area::tools::MissObstacleRootCause cause) {
        PrintKeyValueLine(
            Colorize(passable_area::tools::ToString(cause),
                     MissRootCauseColor(cause).c_str(), style),
            std::to_string(summary.root_cause_counts[static_cast<int>(cause)]));
      };
  print_root_cause(
      passable_area::tools::MissObstacleRootCause::kNoSamplesInRoi);
  print_root_cause(passable_area::tools::MissObstacleRootCause::
                       kNoFrontendObstacleSuspicion);
  print_root_cause(
      passable_area::tools::MissObstacleRootCause::kObstacleEvidenceTooLow);
  print_root_cause(
      passable_area::tools::MissObstacleRootCause::kOutputHeightGateNotMet);
  print_root_cause(passable_area::tools::MissObstacleRootCause::
                       kNoObstacleSourceSamplesInRoi);
  print_root_cause(
      passable_area::tools::MissObstacleRootCause::kUnknownOrMixed);
  print_root_cause(
      passable_area::tools::MissObstacleRootCause::kReasonerNotBlocked);

  PrintSectionHeader("Ranked Frames", style);
  if (summary.ranked_frames.empty()) {
    std::cout << Colorize("╭" + RepeatGlyph("─", kCardColumns) + "╮",
                          "\033[32m", style)
              << '\n';
    std::cout << Colorize("│ No Missed Frames", "\033[1;32m", style) << '\n';
    std::cout << "  obstacle points were observed inside the roi for every "
                 "analyzed frame\n";
    std::cout << Colorize("╰" + RepeatGlyph("─", kCardColumns) + "╯",
                          "\033[32m", style)
              << '\n';
    return;
  }
  for (size_t i = 0; i < summary.ranked_frames.size(); ++i) {
    PrintMissObstacleFrame(summary.ranked_frames[i], static_cast<int>(i + 1),
                           style);
  }
}

std::optional<float> ParseFloatFlagValue(const std::vector<std::string> &args,
                                         const std::string &flag) {
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) {
      return std::stof(args[i + 1]);
    }
  }
  return std::nullopt;
}

std::optional<int> ParseIntFlagValue(const std::vector<std::string> &args,
                                     const std::string &flag) {
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

std::optional<std::string>
ParseStringFlagValue(const std::vector<std::string> &args,
                     const std::string &flag) {
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == flag) {
      return args[i + 1];
    }
  }
  return std::nullopt;
}

std::optional<FalseObstacleReplayArgs>
ParseFalseObstacleReplayArgs(const std::vector<std::string> &args) {
  const auto bag_path = ParseStringFlagValue(args, "--bag");
  const std::string params_file =
      ParseStringFlagValue(args, "--params-file").value_or(DefaultParamsFile());
  const auto start_offset_sec = ParseFloatFlagValue(args, "--start-offset-sec");
  const auto range_x_min = ParseFloatFlagValue(args, "--range-x-min");
  const auto range_x_max = ParseFloatFlagValue(args, "--range-x-max");
  const auto range_y_min = ParseFloatFlagValue(args, "--range-y-min");
  const auto range_y_max = ParseFloatFlagValue(args, "--range-y-max");
  if (!bag_path || !range_x_min || !range_x_max || !range_y_min ||
      !range_y_max) {
    std::cerr << "false-obstacle analysis requires --bag, --range-x-min, "
                 "--range-x-max, "
                 "--range-y-min, and --range-y-max\n";
    return std::nullopt;
  }

  FalseObstacleReplayArgs replay_args;
  replay_args.bag_path = *bag_path;
  replay_args.params_file = params_file;
  replay_args.use_color = !HasFlag(args, "--no-color");
  replay_args.analyzer_config.detection_box =
      passable_area::tools::FalseObstacleDetectionBox{
          *range_x_min, *range_x_max, *range_y_min, *range_y_max};
  if (start_offset_sec) {
    replay_args.start_offset_sec = *start_offset_sec;
    replay_args.use_analysis_window = true;
  }
  if (const auto time_window_sec =
          ParseFloatFlagValue(args, "--time-window-sec")) {
    replay_args.time_window_sec = *time_window_sec;
  }
  if (const auto top_k = ParseIntFlagValue(args, "--top-k")) {
    replay_args.top_k = *top_k;
  }
  if (!(replay_args.analyzer_config.detection_box.x_min <
        replay_args.analyzer_config.detection_box.x_max)) {
    std::cerr
        << "invalid detection box: require --range-x-min < --range-x-max\n";
    return std::nullopt;
  }
  if (!(replay_args.analyzer_config.detection_box.y_min <
        replay_args.analyzer_config.detection_box.y_max)) {
    std::cerr
        << "invalid detection box: require --range-y-min < --range-y-max\n";
    return std::nullopt;
  }
  if (replay_args.top_k <= 0) {
    std::cerr << "invalid --top-k: require top_k > 0\n";
    return std::nullopt;
  }
  if (replay_args.time_window_sec < 0.0f) {
    std::cerr << "invalid --time-window-sec: require time_window_sec >= 0\n";
    return std::nullopt;
  }
  return replay_args;
}

std::optional<MissObstacleReplayArgs>
ParseMissObstacleReplayArgs(const std::vector<std::string> &args) {
  const auto bag_path = ParseStringFlagValue(args, "--bag");
  const std::string params_file =
      ParseStringFlagValue(args, "--params-file").value_or(DefaultParamsFile());
  const auto start_offset_sec = ParseFloatFlagValue(args, "--start-offset-sec");
  const auto roi_x_min = ParseFloatFlagValue(args, "--roi-x-min");
  const auto roi_x_max = ParseFloatFlagValue(args, "--roi-x-max");
  const auto roi_y_min = ParseFloatFlagValue(args, "--roi-y-min");
  const auto roi_y_max = ParseFloatFlagValue(args, "--roi-y-max");
  if (!bag_path || !start_offset_sec || !roi_x_min || !roi_x_max ||
      !roi_y_min || !roi_y_max) {
    std::cerr << "miss-obstacle analysis requires --bag, --start-offset-sec, "
                 "--roi-x-min, "
                 "--roi-x-max, --roi-y-min, and --roi-y-max\n";
    return std::nullopt;
  }

  MissObstacleReplayArgs replay_args;
  replay_args.bag_path = *bag_path;
  replay_args.params_file = params_file;
  replay_args.start_offset_sec = *start_offset_sec;
  replay_args.use_color = !HasFlag(args, "--no-color");
  replay_args.analyzer_config.detection_box =
      passable_area::tools::MissObstacleDetectionBox{*roi_x_min, *roi_x_max,
                                                     *roi_y_min, *roi_y_max};
  if (const auto time_window_sec =
          ParseFloatFlagValue(args, "--time-window-sec")) {
    replay_args.time_window_sec = *time_window_sec;
  }
  if (const auto top_k_cells = ParseIntFlagValue(args, "--top-k-cells")) {
    replay_args.analyzer_config.representative_cell_limit = *top_k_cells;
  }
  if (!(replay_args.analyzer_config.detection_box.x_min <
        replay_args.analyzer_config.detection_box.x_max) ||
      !(replay_args.analyzer_config.detection_box.y_min <
        replay_args.analyzer_config.detection_box.y_max) ||
      replay_args.time_window_sec < 0.0f ||
      replay_args.analyzer_config.representative_cell_limit <= 0) {
    std::cerr << "invalid miss-obstacle analysis arguments\n";
    return std::nullopt;
  }
  return replay_args;
}

std::optional<RoiInspectArgs>
ParseRoiInspectArgs(const std::vector<std::string> &args) {
  const auto bag_path = ParseStringFlagValue(args, "--bag");
  const std::string params_file =
      ParseStringFlagValue(args, "--params-file").value_or(DefaultParamsFile());
  const auto start_offset_sec = ParseFloatFlagValue(args, "--start-offset-sec");
  const auto roi_x_min = ParseFloatFlagValue(args, "--roi-x-min");
  const auto roi_x_max = ParseFloatFlagValue(args, "--roi-x-max");
  const auto roi_y_min = ParseFloatFlagValue(args, "--roi-y-min");
  const auto roi_y_max = ParseFloatFlagValue(args, "--roi-y-max");
  if (!bag_path || !start_offset_sec || !roi_x_min || !roi_x_max ||
      !roi_y_min || !roi_y_max) {
    std::cerr << "roi inspection requires --bag, --start-offset-sec, "
                 "--roi-x-min, --roi-x-max, "
                 "--roi-y-min, and --roi-y-max\n";
    return std::nullopt;
  }

  RoiInspectArgs inspect_args;
  inspect_args.bag_path = *bag_path;
  inspect_args.params_file = params_file;
  inspect_args.start_offset_sec = *start_offset_sec;
  inspect_args.roi_x_min = *roi_x_min;
  inspect_args.roi_x_max = *roi_x_max;
  inspect_args.roi_y_min = *roi_y_min;
  inspect_args.roi_y_max = *roi_y_max;
  if (const auto time_window_sec =
          ParseFloatFlagValue(args, "--time-window-sec")) {
    inspect_args.time_window_sec = *time_window_sec;
  }
  if (!(inspect_args.roi_x_min < inspect_args.roi_x_max) ||
      !(inspect_args.roi_y_min < inspect_args.roi_y_max) ||
      inspect_args.time_window_sec < 0.0f) {
    std::cerr << "invalid roi inspect arguments\n";
    return std::nullopt;
  }
  return inspect_args;
}

std::optional<BagReplaySummary>
RunBagReplay(const std::string &bag_path,
             const passable_area::core::Config &config,
             const passable_area::interfaces::ros::RosTopicConfig &topics) {
  if (!std::filesystem::exists(bag_path)) {
    std::cerr << "bag path does not exist: " << bag_path << '\n';
    return std::nullopt;
  }
  auto reader = OpenBagReader(bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  passable_area::core::Processor processor(config);
  BagReplaySummary summary;
  std::vector<double> processing_ms_samples;
  std::vector<double> input_point_count_samples;

  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == topics.input_cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      clouds.emplace(rclcpp::Time(cloud_msg.header.stamp).nanoseconds(),
                     std::move(cloud_msg));
    } else if (bag_msg->topic_name == topics.odom_topic) {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(),
                    std::move(odom_msg));
    }
  }

  summary.cloud_total_frames = static_cast<int>(clouds.size());
  summary.odom_total_frames = static_cast<int>(odoms.size());

  double unknown_ratio_sum = 0.0;
  int paired_stamp_count = 0;
  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    ++paired_stamp_count;
    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) ||
        !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_map = pose;
    input.input_cloud_in_base = std::move(cloud);
    input_point_count_samples.push_back(
        static_cast<double>(input.input_cloud_in_base.size()));
    const auto start = std::chrono::steady_clock::now();
    const auto output = processor.update(input);
    const double processing_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
    processing_ms_samples.push_back(processing_ms);
    if (!output.valid) {
      ++summary.invalid_frames;
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
    summary.max_missing_sectors =
        std::max(summary.max_missing_sectors, missing);
    const double unknown_ratio =
        static_cast<double>(
            std::count(output.passability.begin(), output.passability.end(),
                       static_cast<int8_t>(PassabilityState::kUnknown))) /
        std::max<size_t>(output.passability.size(), 1U);
    unknown_ratio_sum += unknown_ratio;
    summary.max_unknown_ratio =
        std::max(summary.max_unknown_ratio, unknown_ratio);
  }

  if (summary.paired_frames > 0) {
    summary.avg_unknown_ratio =
        unknown_ratio_sum / static_cast<double>(summary.paired_frames);
  }
  summary.cloud_unpaired_frames =
      std::max(0, summary.cloud_total_frames - paired_stamp_count);
  summary.odom_unpaired_frames =
      std::max(0, summary.odom_total_frames - paired_stamp_count);
  if (!input_point_count_samples.empty()) {
    summary.avg_input_point_count =
        std::accumulate(input_point_count_samples.begin(),
                        input_point_count_samples.end(), 0.0) /
        static_cast<double>(input_point_count_samples.size());
    summary.max_input_point_count = *std::max_element(
        input_point_count_samples.begin(), input_point_count_samples.end());
  }
  if (!processing_ms_samples.empty()) {
    summary.avg_processing_ms =
        std::accumulate(processing_ms_samples.begin(),
                        processing_ms_samples.end(), 0.0) /
        static_cast<double>(processing_ms_samples.size());
    summary.max_processing_ms = *std::max_element(processing_ms_samples.begin(),
                                                  processing_ms_samples.end());
    summary.p95_processing_ms = Percentile(processing_ms_samples, 0.95);
  }
  return summary;
}

std::string DefaultParamsFile() {
  return "/home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/"
         "passable_area.yaml";
}

std::vector<std::string> ParamsFilesForReplay(
    const std::string &primary_params_file) {
  std::vector<std::string> params_files;
  params_files.push_back(primary_params_file);
  const std::filesystem::path primary_path(primary_params_file);
  const auto parent = primary_path.parent_path();
  const auto append_if_distinct_and_exists =
      [&](const std::filesystem::path &candidate) {
        if (candidate.empty() || candidate == primary_path ||
            !std::filesystem::exists(candidate)) {
          return;
        }
        params_files.push_back(candidate.string());
      };
  append_if_distinct_and_exists(parent / "sensors.yaml");
  append_if_distinct_and_exists(parent / "debug.yaml");
  return params_files;
}

std::vector<std::string> DefaultParamsFiles() {
  return {
      "/home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/"
      "passable_area.yaml",
      "/home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/"
      "sensors.yaml",
      "/home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/"
      "debug.yaml",
  };
}

std::optional<passable_area::interfaces::ros::RosNodeParams>
LoadNodeParamsFromParamsFiles(const std::vector<std::string> &params_files) {
  if (params_files.empty()) {
    std::cerr << "no params files provided\n";
    return std::nullopt;
  }
  for (const auto &params_file : params_files) {
    if (!std::filesystem::exists(params_file)) {
      std::cerr << "params file does not exist: " << params_file << '\n';
      return std::nullopt;
    }
  }

  rclcpp::NodeOptions options;
  std::vector<std::string> arguments = {"--ros-args"};
  arguments.reserve(1 + params_files.size() * 2);
  for (const auto &params_file : params_files) {
    arguments.push_back("--params-file");
    arguments.push_back(params_file);
  }
  options.arguments(arguments);
  auto node = std::make_shared<rclcpp::Node>("passable_area", options);
  return passable_area::interfaces::ros::RosParamLoader{}.load(*node);
}

std::optional<passable_area::tools::FalseObstacleBagSummary>
RunFalseObstacleReplay(const FalseObstacleReplayArgs &args) {
  if (!std::filesystem::exists(args.bag_path)) {
    std::cerr << "bag path does not exist: " << args.bag_path << '\n';
    return std::nullopt;
  }

  const auto node_params =
      LoadNodeParamsFromParamsFiles(ParamsFilesForReplay(args.params_file));
  if (!node_params) {
    return std::nullopt;
  }
  const auto &config = node_params->config;
  const auto &topics = node_params->topics;

  auto reader = OpenBagReader(args.bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, int64_t> cloud_bag_times;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  std::optional<int64_t> bag_start_time;
  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    if (!bag_start_time.has_value()) {
      bag_start_time = bag_msg->time_stamp;
    } else {
      bag_start_time = std::min(*bag_start_time, bag_msg->time_stamp);
    }
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == topics.input_cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      const int64_t stamp = rclcpp::Time(cloud_msg.header.stamp).nanoseconds();
      clouds.emplace(stamp, std::move(cloud_msg));
      cloud_bag_times.emplace(stamp, bag_msg->time_stamp);
    } else if (bag_msg->topic_name == topics.odom_topic) {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(),
                    std::move(odom_msg));
    }
  }

  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  passable_area::core::Processor processor(config);
  passable_area::tools::FalseObstacleAnalyzer analyzer(config,
                                                       args.analyzer_config);

  int total_frames = 0;
  int rear_dropout_frames = 0;
  int current_candidate_run = 0;
  int longest_candidate_run = 0;
  std::vector<passable_area::tools::FalseObstacleFrameAnalysis>
      candidate_frames;
  const double half_window_sec =
      static_cast<double>(args.time_window_sec) * 0.5;

  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) ||
        !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_map = pose;
    input.input_cloud_in_base = std::move(cloud);
    const auto output = processor.update(input);
    if (!output.valid) {
      continue;
    }

    const auto bag_time_it = cloud_bag_times.find(stamp);
    std::optional<double> start_offset_sec;
    if (bag_start_time.has_value() && bag_time_it != cloud_bag_times.end()) {
      start_offset_sec =
          static_cast<double>(bag_time_it->second - *bag_start_time) * 1e-9;
    }
    const bool in_analysis_window =
        !args.use_analysis_window ||
        (start_offset_sec.has_value() &&
         std::abs(*start_offset_sec -
                  static_cast<double>(args.start_offset_sec)) <=
             half_window_sec);
    if (!in_analysis_window) {
      continue;
    }

    ++total_frames;
    if (output.observability.rear_dropout) {
      ++rear_dropout_frames;
    }
    const auto analysis = analyzer.analyzeFrame(output);
    if (analysis) {
      auto frame_analysis = *analysis;
      if (start_offset_sec.has_value()) {
        frame_analysis.start_offset_sec = *start_offset_sec;
      }
      candidate_frames.push_back(std::move(frame_analysis));
      ++current_candidate_run;
      longest_candidate_run =
          std::max(longest_candidate_run, current_candidate_run);
    } else {
      current_candidate_run = 0;
    }
  }

  return analyzer.buildSummary(total_frames, rear_dropout_frames,
                               longest_candidate_run,
                               std::move(candidate_frames), args.top_k);
}

std::optional<passable_area::tools::MissObstacleBagSummary>
RunMissObstacleReplay(const MissObstacleReplayArgs &args) {
  if (!std::filesystem::exists(args.bag_path)) {
    std::cerr << "bag path does not exist: " << args.bag_path << '\n';
    return std::nullopt;
  }

  const auto node_params =
      LoadNodeParamsFromParamsFiles(ParamsFilesForReplay(args.params_file));
  if (!node_params) {
    return std::nullopt;
  }
  const auto &config = node_params->config;
  const auto &topics = node_params->topics;

  auto reader = OpenBagReader(args.bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, int64_t> cloud_bag_times;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  std::optional<int64_t> bag_start_time;
  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    if (!bag_start_time.has_value()) {
      bag_start_time = bag_msg->time_stamp;
    } else {
      bag_start_time = std::min(*bag_start_time, bag_msg->time_stamp);
    }
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == topics.input_cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      const int64_t stamp = rclcpp::Time(cloud_msg.header.stamp).nanoseconds();
      clouds.emplace(stamp, std::move(cloud_msg));
      cloud_bag_times.emplace(stamp, bag_msg->time_stamp);
    } else if (bag_msg->topic_name == topics.odom_topic) {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(),
                    std::move(odom_msg));
    }
  }

  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  passable_area::core::FramePreprocessor preprocessor(config);
  passable_area::core::Processor processor(config);
  passable_area::tools::MissObstacleAnalyzer analyzer(config,
                                                      args.analyzer_config);

  int total_frames = 0;
  std::vector<passable_area::tools::MissObstacleFrameAnalysis> candidate_frames;
  const double half_window_sec =
      static_cast<double>(args.time_window_sec) * 0.5;

  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    const auto bag_time_it = cloud_bag_times.find(stamp);
    if (!bag_start_time.has_value() || bag_time_it == cloud_bag_times.end()) {
      continue;
    }
    const double start_offset_sec =
        static_cast<double>(bag_time_it->second - *bag_start_time) * 1e-9;
    const bool in_analysis_window =
        std::abs(start_offset_sec -
                 static_cast<double>(args.start_offset_sec)) <= half_window_sec;

    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) ||
        !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_map = pose;
    input.input_cloud_in_base = cloud;

    ProcessedFrame processed_frame;
    if (!preprocessor.process(input, processed_frame)) {
      continue;
    }
    const auto output = processor.update(input);
    if (!output.valid) {
      continue;
    }

    if (!in_analysis_window) {
      continue;
    }

    ++total_frames;
    const auto analysis = analyzer.analyzeFrame(output, processed_frame);
    if (!analysis) {
      continue;
    }
    auto frame_analysis = *analysis;
    frame_analysis.start_offset_sec = start_offset_sec;
    candidate_frames.push_back(std::move(frame_analysis));
  }

  return analyzer.buildSummary(total_frames, std::move(candidate_frames));
}

void PrintRoiFrameInspection(
    const FrameOutput &output, const passable_area::core::Pose3D &base_pose,
    const passable_area::core::ProcessedFrame &processed_frame,
    double start_offset_sec, const RoiInspectArgs &args,
    const passable_area::core::Config &config) {
  std::cout << std::fixed << std::setprecision(3);
  const float yaw = std::atan2(
      2.0f * (base_pose.orientation.w() * base_pose.orientation.z() +
              base_pose.orientation.x() * base_pose.orientation.y()),
      1.0f - 2.0f * (base_pose.orientation.y() * base_pose.orientation.y() +
                     base_pose.orientation.z() * base_pose.orientation.z()));
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  int roi_sample_count = 0;
  float roi_sample_min_z = std::numeric_limits<float>::infinity();
  float roi_sample_max_z = -std::numeric_limits<float>::infinity();
  for (const auto &sample : processed_frame.map_samples) {
    const float dx = sample.point_in_map.x - base_pose.position.x();
    const float dy = sample.point_in_map.y - base_pose.position.y();
    const float base_gravity_x = cos_yaw * dx + sin_yaw * dy;
    const float base_gravity_y = -sin_yaw * dx + cos_yaw * dy;
    if (base_gravity_x < args.roi_x_min || base_gravity_x > args.roi_x_max ||
        base_gravity_y < args.roi_y_min || base_gravity_y > args.roi_y_max) {
      continue;
    }
    ++roi_sample_count;
    const float relative_z = sample.point_in_map.z - base_pose.position.z();
    roi_sample_min_z = std::min(roi_sample_min_z, relative_z);
    roi_sample_max_z = std::max(roi_sample_max_z, relative_z);
  }
  std::cout << "frame_offset_sec=" << start_offset_sec << " base_pose=("
            << base_pose.position.x() << ", " << base_pose.position.y() << ", "
            << base_pose.position.z() << ") roi_frame=base_gravity"
            << " roi_samples=" << roi_sample_count;
  if (roi_sample_count > 0) {
    std::cout << " roi_sample_relative_z=[" << roi_sample_min_z << ", "
              << roi_sample_max_z << "]";
  }
  std::cout << "\n";
  int roi_total = 0;
  int roi_impassable = 0;
  int roi_passable = 0;
  int roi_unknown = 0;
  int roi_rejected = 0;
  int roi_upper = 0;
  float roi_max_obstacle_evidence = 0.0f;
  float roi_max_protrusion_evidence = 0.0f;
  float roi_max_overhead_evidence = 0.0f;
  int roi_protrusion_publish_cells = 0;
  int roi_low_clearance_bridge_cells = 0;
  std::vector<int> roi_sample_count_by_cell(
      static_cast<size_t>(output.rows * output.cols), 0);
  std::vector<float> roi_sample_min_z_by_cell(
      static_cast<size_t>(output.rows * output.cols),
      std::numeric_limits<float>::infinity());
  std::vector<float> roi_sample_max_z_by_cell(
      static_cast<size_t>(output.rows * output.cols),
      -std::numeric_limits<float>::infinity());
  std::vector<float> roi_sample_min_base_z_by_cell(
      static_cast<size_t>(output.rows * output.cols),
      std::numeric_limits<float>::infinity());
  std::vector<float> roi_sample_max_base_z_by_cell(
      static_cast<size_t>(output.rows * output.cols),
      -std::numeric_limits<float>::infinity());
  for (const auto &sample : processed_frame.map_samples) {
    const float fx =
        (sample.point_in_map.x - output.origin.x()) / output.resolution;
    const float fy =
        (sample.point_in_map.y - output.origin.y()) / output.resolution;
    const int col = static_cast<int>(std::floor(fx));
    const int row = static_cast<int>(std::floor(fy));
    if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
      continue;
    }
    const int cell = row * output.cols + col;
    const float dx = sample.point_in_map.x - base_pose.position.x();
    const float dy = sample.point_in_map.y - base_pose.position.y();
    const float base_gravity_x = cos_yaw * dx + sin_yaw * dy;
    const float base_gravity_y = -sin_yaw * dx + cos_yaw * dy;
    if (base_gravity_x < args.roi_x_min || base_gravity_x > args.roi_x_max ||
        base_gravity_y < args.roi_y_min || base_gravity_y > args.roi_y_max) {
      continue;
    }
    ++roi_sample_count_by_cell[static_cast<size_t>(cell)];
    roi_sample_min_z_by_cell[static_cast<size_t>(cell)] =
        std::min(roi_sample_min_z_by_cell[static_cast<size_t>(cell)],
                 sample.point_in_map.z);
    roi_sample_max_z_by_cell[static_cast<size_t>(cell)] =
        std::max(roi_sample_max_z_by_cell[static_cast<size_t>(cell)],
                 sample.point_in_map.z);
    roi_sample_min_base_z_by_cell[static_cast<size_t>(cell)] =
        std::min(roi_sample_min_base_z_by_cell[static_cast<size_t>(cell)],
                 sample.point_in_base.z);
    roi_sample_max_base_z_by_cell[static_cast<size_t>(cell)] =
        std::max(roi_sample_max_base_z_by_cell[static_cast<size_t>(cell)],
                 sample.point_in_base.z);
  }
  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const float map_x = output.origin.x() +
                           (static_cast<float>(col) + 0.5f) * output.resolution;
      const float map_y = output.origin.y() +
                           (static_cast<float>(row) + 0.5f) * output.resolution;
      const float dx = map_x - base_pose.position.x();
      const float dy = map_y - base_pose.position.y();
      const float base_gravity_x = cos_yaw * dx + sin_yaw * dy;
      const float base_gravity_y = -sin_yaw * dx + cos_yaw * dy;
      if (base_gravity_x < args.roi_x_min || base_gravity_x > args.roi_x_max ||
          base_gravity_y < args.roi_y_min || base_gravity_y > args.roi_y_max) {
        continue;
      }
      const int idx = row * output.cols + col;
      ++roi_total;
      if (output.passability[idx] ==
          static_cast<int8_t>(PassabilityState::kImpassable)) {
        ++roi_impassable;
      } else if (output.passability[idx] ==
                 static_cast<int8_t>(PassabilityState::kPassable)) {
        ++roi_passable;
      } else {
        ++roi_unknown;
      }
      if (output.obstacle_suspicious[idx] != 0U &&
          output.obstacle_candidate_cell[idx] == 0U) {
        ++roi_rejected;
      }
      if (output.block_reason[idx] != 0U) {
        ++roi_upper;
      }
      roi_max_obstacle_evidence =
          std::max(roi_max_obstacle_evidence, output.obstacle_evidence[idx]);
      const float protrusion_evidence =
          output.protrusion_evidence.empty() ? 0.0f
                                             : output.protrusion_evidence[idx];
      const float overhead_evidence =
          output.overhead_evidence.empty() ? 0.0f : output.overhead_evidence[idx];
      const uint8_t block_reason =
          output.block_reason.empty() ? 0U : output.block_reason[idx];
      roi_max_protrusion_evidence =
          std::max(roi_max_protrusion_evidence, protrusion_evidence);
      roi_max_overhead_evidence =
          std::max(roi_max_overhead_evidence, overhead_evidence);
      if (protrusion_evidence >= config.obstacle_points_min_evidence) {
        ++roi_protrusion_publish_cells;
      }
      if (IsLowClearanceBridgeEligible(config, block_reason,
                                       output.clearance[idx],
                                       overhead_evidence)) {
        ++roi_low_clearance_bridge_cells;
      }
    }
  }
  std::cout << "roi_summary total=" << roi_total
            << " impassable=" << roi_impassable << " passable=" << roi_passable
            << " unknown=" << roi_unknown << " rejected=" << roi_rejected
            << " upper_support=" << roi_upper
            << " max_obstacle_evidence=" << roi_max_obstacle_evidence
            << " max_protrusion_evidence=" << roi_max_protrusion_evidence
            << " max_overhead_evidence=" << roi_max_overhead_evidence
            << " protrusion_publish_cells=" << roi_protrusion_publish_cells
            << " low_clearance_bridge_cells="
            << roi_low_clearance_bridge_cells << "\n";

  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const float map_x = output.origin.x() +
                           (static_cast<float>(col) + 0.5f) * output.resolution;
      const float map_y = output.origin.y() +
                           (static_cast<float>(row) + 0.5f) * output.resolution;
      const float dx = map_x - base_pose.position.x();
      const float dy = map_y - base_pose.position.y();
      const float base_gravity_x = cos_yaw * dx + sin_yaw * dy;
      const float base_gravity_y = -sin_yaw * dx + cos_yaw * dy;
      if (base_gravity_x < args.roi_x_min || base_gravity_x > args.roi_x_max ||
          base_gravity_y < args.roi_y_min || base_gravity_y > args.roi_y_max) {
        continue;
      }
      const int idx = row * output.cols + col;
      const float protrusion_evidence =
          output.protrusion_evidence.empty() ? 0.0f
                                             : output.protrusion_evidence[idx];
      const float overhead_evidence =
          output.overhead_evidence.empty() ? 0.0f : output.overhead_evidence[idx];
      const uint8_t block_reason =
          output.block_reason.empty() ? 0U : output.block_reason[idx];
      const uint8_t publish_status =
          output.obstacle_point_publish_status.empty()
              ? 0U
              : output.obstacle_point_publish_status[idx];
      const bool low_clearance_bridge =
          IsLowClearanceBridgeEligible(config, block_reason,
                                       output.clearance[idx],
                                       overhead_evidence);
      const std::string publish_path = PublishPathFromStatus(
          static_cast<passable_area::core::ObstaclePointPublishStatus>(
              publish_status));
      std::cout
          << "  cell base_x=" << base_gravity_x << " base_y=" << base_gravity_y
          << " map_x=" << map_x << " map_y=" << map_y << " sample_count="
          << roi_sample_count_by_cell[static_cast<size_t>(idx)]
          << " sample_min_z="
          << roi_sample_min_z_by_cell[static_cast<size_t>(idx)]
          << " sample_max_z="
          << roi_sample_max_z_by_cell[static_cast<size_t>(idx)]
          << " sample_base_z=["
          << roi_sample_min_base_z_by_cell[static_cast<size_t>(idx)] << ","
          << roi_sample_max_base_z_by_cell[static_cast<size_t>(idx)] << "]"
          << " passability=" << static_cast<int>(output.passability[idx])
          << " support_h=" << output.support_height[idx]
          << " overhead_h=" << output.overhead_height[idx]
          << " obstacle_evidence=" << output.obstacle_evidence[idx]
          << " protrusion_evidence=" << protrusion_evidence
          << " overhead_evidence=" << overhead_evidence
          << " clearance=" << output.clearance[idx]
          << " support_continuity=" << output.support_continuity[idx]
          << " block_reason="
          << ToBlockReasonString(
                 static_cast<passable_area::core::BlockReason>(block_reason))
          << " publish_status="
          << ToObstaclePointPublishStatusString(
                 static_cast<passable_area::core::ObstaclePointPublishStatus>(
                     publish_status))
          << " publish_path=" << publish_path
          << " low_clearance_bridge_hit="
          << static_cast<int>(low_clearance_bridge)
          << " raw_min_z=" << output.raw_sample_min_z[idx]
          << " raw_max_z=" << output.raw_sample_max_z[idx]
          << " raw_count=" << output.raw_sample_count[idx]
          << " filtered_min_z=" << output.filtered_sample_min_z[idx]
          << " filtered_max_z=" << output.filtered_sample_max_z[idx]
          << " filtered_count=" << output.filtered_sample_count[idx]
          << " suspicious=" << static_cast<int>(output.obstacle_suspicious[idx])
          << " candidate=" << static_cast<int>(output.obstacle_candidate_cell[idx])
          << " coverage=" << output.coverage_confidence[idx]
          << " support_conf=" << output.support_confidence[idx] << "\n";
    }
  }
}

std::optional<int> RunRoiInspect(const RoiInspectArgs &args) {
  if (!std::filesystem::exists(args.bag_path)) {
    std::cerr << "bag path does not exist: " << args.bag_path << '\n';
    return std::nullopt;
  }
  const auto node_params =
      LoadNodeParamsFromParamsFiles(ParamsFilesForReplay(args.params_file));
  if (!node_params) {
    return std::nullopt;
  }
  const auto &config = node_params->config;
  const auto &topics = node_params->topics;

  auto reader = OpenBagReader(args.bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, int64_t> cloud_bag_times;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  std::optional<int64_t> bag_start_time;
  while (reader->has_next()) {
    auto bag_msg = reader->read_next();
    if (!bag_start_time.has_value()) {
      bag_start_time = bag_msg->time_stamp;
    } else {
      bag_start_time = std::min(*bag_start_time, bag_msg->time_stamp);
    }
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == topics.input_cloud_topic) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      const int64_t stamp = rclcpp::Time(cloud_msg.header.stamp).nanoseconds();
      clouds.emplace(stamp, std::move(cloud_msg));
      cloud_bag_times.emplace(stamp, bag_msg->time_stamp);
    } else if (bag_msg->topic_name == topics.odom_topic) {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(),
                    std::move(odom_msg));
    }
  }

  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  passable_area::core::FramePreprocessor preprocessor(config);
  passable_area::core::Processor processor(config);
  int printed_frames = 0;
  const double half_window_sec =
      static_cast<double>(args.time_window_sec) * 0.5;

  for (const auto &[stamp, cloud_msg] : clouds) {
    auto odom_it = odoms.find(stamp);
    if (odom_it == odoms.end()) {
      continue;
    }
    const auto bag_time_it = cloud_bag_times.find(stamp);
    if (!bag_start_time.has_value() || bag_time_it == cloud_bag_times.end()) {
      continue;
    }
    const double start_offset_sec =
        static_cast<double>(bag_time_it->second - *bag_start_time) * 1e-9;

    passable_area::core::PointCloud cloud;
    passable_area::core::Pose3D pose;
    if (!cloud_converter.fromRos(cloud_msg, cloud) ||
        !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_map = pose;
    input.input_cloud_in_base = std::move(cloud);
    passable_area::core::ProcessedFrame processed_frame;
    if (!preprocessor.process(input, processed_frame)) {
      continue;
    }
    const auto output = processor.update(input);
    if (!output.valid) {
      continue;
    }
    if (std::abs(start_offset_sec -
                 static_cast<double>(args.start_offset_sec)) >
        half_window_sec) {
      continue;
    }
    PrintRoiFrameInspection(output, pose, processed_frame, start_offset_sec,
                            args, config);
    ++printed_frames;
  }
  return printed_frames;
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::vector<std::string> args(argv + 1, argv + argc);

  if (HasFlag(args, "--analyze-missed-obstacles")) {
    const auto replay_args = ParseMissObstacleReplayArgs(args);
    if (!replay_args) {
      rclcpp::shutdown();
      return 1;
    }
    const auto summary = RunMissObstacleReplay(*replay_args);
    if (!summary) {
      rclcpp::shutdown();
      return 1;
    }
    PrintMissObstacleSummary(*summary, *replay_args,
                             TerminalStyle{replay_args->use_color});
    rclcpp::shutdown();
    return 0;
  }

  if (HasFlag(args, "--inspect-roi")) {
    const auto inspect_args = ParseRoiInspectArgs(args);
    if (!inspect_args) {
      rclcpp::shutdown();
      return 1;
    }
    const auto printed_frames = RunRoiInspect(*inspect_args);
    if (!printed_frames) {
      rclcpp::shutdown();
      return 1;
    }
    std::cout << "printed_frames=" << *printed_frames << '\n';
    rclcpp::shutdown();
    return 0;
  }

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
    PrintFalseObstacleSummary(*summary, replay_args->bag_path,
                              replay_args->params_file, *replay_args,
                              TerminalStyle{replay_args->use_color});
    rclcpp::shutdown();
    return 0;
  }

  if (HasFlag(args, "--benchmark-timing") ||
      (args.size() >= 2 && args[0] == "--bag")) {
    const auto bag_path = ParseStringFlagValue(args, "--bag");
    if (!bag_path) {
      std::cerr << "timing benchmark requires --bag <path>\n";
      rclcpp::shutdown();
      return 1;
    }
    const std::string params_file = ParseStringFlagValue(args, "--params-file")
                                        .value_or(DefaultParamsFile());
    const auto node_params =
        LoadNodeParamsFromParamsFiles(ParamsFilesForReplay(params_file));
    if (!node_params) {
      rclcpp::shutdown();
      return 1;
    }
    const auto summary =
        RunBagReplay(*bag_path, node_params->config, node_params->topics);
    if (!summary) {
      rclcpp::shutdown();
      return 1;
    }
    std::cout << "bag=" << *bag_path << " params_file=" << params_file
              << " cloud_total_frames=" << summary->cloud_total_frames
              << " odom_total_frames=" << summary->odom_total_frames
              << " paired_frames=" << summary->paired_frames
              << " cloud_unpaired_frames=" << summary->cloud_unpaired_frames
              << " odom_unpaired_frames=" << summary->odom_unpaired_frames
              << " invalid_frames=" << summary->invalid_frames
              << " rear_dropout_frames=" << summary->rear_dropout_frames
              << " max_missing_sectors=" << summary->max_missing_sectors
              << " avg_unknown_ratio=" << std::fixed << std::setprecision(3)
              << summary->avg_unknown_ratio
              << " max_unknown_ratio=" << summary->max_unknown_ratio
              << " avg_input_points=" << std::setprecision(1)
              << summary->avg_input_point_count
              << " max_input_points=" << std::setprecision(0)
              << summary->max_input_point_count
              << " avg_processing_ms=" << std::setprecision(3)
              << summary->avg_processing_ms
              << " max_processing_ms=" << summary->max_processing_ms
              << " p95_processing_ms=" << summary->p95_processing_ms << '\n';
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
