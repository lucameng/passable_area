#include "passable_area/interfaces/ros/converters/odom_converter.hpp"
#include "passable_area/interfaces/ros/converters/pointcloud_converter.hpp"
#include "passable_area/interfaces/ros/params/ros_param_loader.hpp"
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

std::string DefaultParamsFile();
std::optional<passable_area::core::Config> LoadConfigFromParamsFile(
    const std::string &params_file);

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
  int top_k = 10;
  bool use_color = true;
};

struct TerminalStyle {
  bool use_color = true;
};

std::string Colorize(const std::string &text, const char *ansi_code, const TerminalStyle &style) {
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
    case passable_area::tools::FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity:
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
    case passable_area::core::ObservabilityState::kPartiallyObserved:
      return "\033[33m";
    case passable_area::core::ObservabilityState::kMissingByDropout:
      return "\033[31m";
  }
  return "\033[37m";
}

std::string BoolBadge(bool value, const TerminalStyle &style) {
  return Colorize(value ? "true" : "false", value ? "\033[32m" : "\033[90m", style);
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
  const size_t idx = std::min(samples.size() - 1,
                              static_cast<size_t>(std::floor(p * (samples.size() - 1))));
  return samples[idx];
}

std::string FormatDetectionBox(const passable_area::tools::FalseObstacleDetectionBox &box) {
  std::ostringstream oss;
  oss << "x[" << std::fixed << std::setprecision(2) << box.x_min << ", " << box.x_max << "] y["
      << box.y_min << ", " << box.y_max << "]";
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
  std::cout << Colorize("╔" + RepeatGlyph("═", kCardColumns) + "╗", "\033[36m", style)
            << '\n';
  std::cout << Colorize("║ " + title, "\033[1;36m", style) << '\n';
  std::cout << Colorize("╚" + RepeatGlyph("═", kCardColumns) + "╝", "\033[36m", style)
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
  std::cout << '\n'
            << Colorize(line, "\033[36m", style)
            << '\n';
}

void PrintKeyValueLine(const std::string &label, const std::string &value) {
  std::cout << "  " << label << ": " << value << '\n';
}

void PrintDetectionBox(const passable_area::tools::FalseObstacleDetectionBox &box,
                       const TerminalStyle &style) {
  PrintKeyValueLine("detection_box", Colorize(FormatDetectionBox(box), "\033[36m", style));
}

void PrintFalseObstacleFrame(const passable_area::tools::FalseObstacleFrameAnalysis &frame,
                             int rank, const TerminalStyle &style) {
  const auto class_label = Colorize(passable_area::tools::ToString(frame.classification),
                                    RootCauseColor(frame.classification).c_str(), style);
  std::cout << '\n'
            << Colorize("╔" + RepeatGlyph("═", kCardColumns) + "╗", "\033[36m", style)
            << '\n';
  std::cout << Colorize("║ frame " + std::to_string(rank), "\033[1;36m", style)
            << "  " << class_label
            << "  severity=" << Colorize(FormatFloat(frame.severity), "\033[1;33m", style)
            << '\n';
  std::cout << Colorize("╟" + RepeatGlyph("─", kCardColumns) + "╢", "\033[36m", style)
            << '\n';
  PrintKeyValueLine("stamp", std::to_string(frame.stamp));
  PrintKeyValueLine("start_offset", FormatFloat(frame.start_offset_sec, 3) + " s");
  PrintKeyValueLine("in_box_obstacle_points", std::to_string(frame.in_box_obstacle_point_count));
  PrintKeyValueLine("hotspot_count", std::to_string(frame.hotspots.size()));
  PrintKeyValueLine("frame_partial", BoolBadge(frame.frame_partial, style));
  PrintKeyValueLine("rear_dropout", BoolBadge(frame.rear_dropout, style));
  PrintKeyValueLine("max_obstacle_evidence", FormatFloat(frame.max_local_obstacle_evidence));
  PrintKeyValueLine("min_clearance", FormatFloat(frame.min_local_clearance));
  PrintKeyValueLine("min_support_continuity", FormatFloat(frame.min_local_support_continuity));
  PrintKeyValueLine("rejected_suspicious_cells",
                    std::to_string(frame.rejected_suspicious_cell_count));

  for (size_t i = 0; i < frame.hotspots.size(); ++i) {
    const auto &hotspot = frame.hotspots[i];
    const std::string hotspot_class =
        Colorize(passable_area::tools::ToString(hotspot.classification),
                 RootCauseColor(hotspot.classification).c_str(), style);
    const std::string observability = hotspot.has_observability
                                          ? Colorize(passable_area::tools::ToString(
                                                         hotspot.observability_state),
                                                     ObservabilityColor(hotspot.observability_state)
                                                         .c_str(),
                                                     style)
                                          : "Unavailable";
    std::cout << "  "
              << Colorize("• hotspot " + std::to_string(i + 1), "\033[1;35m", style) << '\n';
    std::cout << "    pos: (" << FormatFloat(hotspot.x) << ", " << FormatFloat(hotspot.y)
              << ")  z: [" << FormatFloat(hotspot.min_z) << ", " << FormatFloat(hotspot.max_z)
              << "]  obstacle_points: " << hotspot.obstacle_point_count
              << "  severity: " << Colorize(FormatFloat(hotspot.severity), "\033[1;33m", style)
              << '\n';
    std::cout << "    class: " << hotspot_class << "  observability: " << observability
              << '\n';
    std::cout << "    why: " << Colorize(hotspot.explanation, "\033[1;37m", style) << '\n';
    std::cout << "    obstacle_evidence: " << FormatFloat(hotspot.obstacle_evidence)
              << "  clearance: " << FormatFloat(hotspot.clearance)
              << "  support_continuity: " << FormatFloat(hotspot.support_continuity) << '\n';
    std::cout << "    upper_support_cell: " << (hotspot.upper_support_cell ? "true" : "false")
              << "  neighbor_upper_support_count: "
              << std::to_string(hotspot.neighbor_upper_support_count)
              << "  rejected_by_neighbor_support: "
              << (hotspot.obstacle_rejected_by_neighbor_support ? "true" : "false") << '\n';
  }
  std::cout << Colorize("╚" + RepeatGlyph("═", kCardColumns) + "╝", "\033[36m", style)
            << '\n';
}

void PrintFalseObstacleSummary(const passable_area::tools::FalseObstacleBagSummary &summary,
                               const std::string &bag_path, const std::string &params_file,
                               int top_k, const TerminalStyle &style) {
  PrintBanner("False Obstacle Offline Analysis", style);
  PrintSectionHeader("Run", style);
  PrintKeyValueLine("bag", bag_path);
  PrintKeyValueLine("params_file", params_file);
  PrintDetectionBox(summary.detection_box, style);
  PrintKeyValueLine("top_k", std::to_string(top_k));

  const double candidate_ratio =
      summary.total_frames > 0
          ? static_cast<double>(summary.candidate_frames) / static_cast<double>(summary.total_frames)
          : 0.0;
  PrintSectionHeader("Summary", style);
  PrintKeyValueLine("total_frames", std::to_string(summary.total_frames));
  PrintKeyValueLine("rear_dropout_frames", std::to_string(summary.rear_dropout_frames));
  PrintKeyValueLine("suspicious_frames",
                    Colorize(std::to_string(summary.candidate_frames),
                             summary.candidate_frames > 0 ? "\033[1;31m" : "\033[1;32m", style));
  PrintKeyValueLine("suspicious_ratio", FormatFloat(candidate_ratio, 3));
  PrintKeyValueLine("longest_consecutive_run",
                    std::to_string(summary.longest_consecutive_candidate_run));

  PrintSectionHeader("Root Causes", style);
  const auto print_root_cause = [&](passable_area::tools::FalseObstacleRootCause cause) {
    const std::string label = passable_area::tools::ToString(cause);
    const std::string value = std::to_string(summary.root_cause_counts[static_cast<int>(cause)]);
    PrintKeyValueLine(Colorize(label, RootCauseColor(cause).c_str(), style), value);
  };
  print_root_cause(passable_area::tools::FalseObstacleRootCause::kClearanceDriven);
  print_root_cause(passable_area::tools::FalseObstacleRootCause::kObstacleEvidenceDriven);
  print_root_cause(passable_area::tools::FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity);
  print_root_cause(passable_area::tools::FalseObstacleRootCause::kObservabilityInfluenced);
  print_root_cause(passable_area::tools::FalseObstacleRootCause::kUnknownOrMixed);

  PrintSectionHeader("Ranked Frames", style);
  if (summary.ranked_frames.empty()) {
    std::cout << Colorize("╭" + RepeatGlyph("─", kCardColumns) + "╮", "\033[32m", style)
              << '\n';
    std::cout << Colorize("│ No Candidate Frames", "\033[1;32m", style) << '\n';
    std::cout << "  no obstacle_points were found inside the detection box\n";
    std::cout << Colorize("╰" + RepeatGlyph("─", kCardColumns) + "╯", "\033[32m", style)
              << '\n';
    return;
  }
  for (size_t i = 0; i < summary.ranked_frames.size(); ++i) {
    PrintFalseObstacleFrame(summary.ranked_frames[i], static_cast<int>(i + 1), style);
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
  const std::string params_file =
      ParseStringFlagValue(args, "--params-file").value_or(DefaultParamsFile());
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
  replay_args.params_file = params_file;
  replay_args.use_color = !HasFlag(args, "--no-color");
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

std::optional<BagReplaySummary> RunBagReplay(const std::string &bag_path,
                                             const passable_area::core::Config &config) {
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
  passable_area::core::Processor processor(config);
  BagReplaySummary summary;
  std::vector<double> processing_ms_samples;
  std::vector<double> input_point_count_samples;

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
    if (!cloud_converter.fromRos(cloud_msg, cloud) || !odom_converter.fromRos(odom_it->second, pose)) {
      continue;
    }
    FrameInput input;
    input.stamp = stamp;
    input.base_pose_in_odom = pose;
    input.input_cloud_in_base = std::move(cloud);
    input_point_count_samples.push_back(static_cast<double>(input.input_cloud_in_base.size()));
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
    summary.max_missing_sectors = std::max(summary.max_missing_sectors, missing);
    const double unknown_ratio =
        static_cast<double>(std::count(output.passability.begin(), output.passability.end(),
                                       static_cast<int8_t>(PassabilityState::kUnknown))) /
        std::max<size_t>(output.passability.size(), 1U);
    unknown_ratio_sum += unknown_ratio;
    summary.max_unknown_ratio = std::max(summary.max_unknown_ratio, unknown_ratio);
  }

  if (summary.paired_frames > 0) {
    summary.avg_unknown_ratio = unknown_ratio_sum / static_cast<double>(summary.paired_frames);
  }
  summary.cloud_unpaired_frames = std::max(0, summary.cloud_total_frames - paired_stamp_count);
  summary.odom_unpaired_frames = std::max(0, summary.odom_total_frames - paired_stamp_count);
  if (!input_point_count_samples.empty()) {
    summary.avg_input_point_count =
        std::accumulate(input_point_count_samples.begin(), input_point_count_samples.end(), 0.0) /
        static_cast<double>(input_point_count_samples.size());
    summary.max_input_point_count =
        *std::max_element(input_point_count_samples.begin(), input_point_count_samples.end());
  }
  if (!processing_ms_samples.empty()) {
    summary.avg_processing_ms =
        std::accumulate(processing_ms_samples.begin(), processing_ms_samples.end(), 0.0) /
        static_cast<double>(processing_ms_samples.size());
    summary.max_processing_ms =
        *std::max_element(processing_ms_samples.begin(), processing_ms_samples.end());
    summary.p95_processing_ms = Percentile(processing_ms_samples, 0.95);
  }
  return summary;
}

std::string DefaultParamsFile() {
  return "/home/deep/deeprobotics/passable_humble_ws/src/passable_area/config/passable_area.yaml";
}

std::optional<passable_area::core::Config> LoadConfigFromParamsFile(const std::string &params_file) {
  if (!std::filesystem::exists(params_file)) {
    std::cerr << "params file does not exist: " << params_file << '\n';
    return std::nullopt;
  }

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_file});
  auto node = std::make_shared<rclcpp::Node>("passable_area", options);
  return passable_area::interfaces::ros::RosParamLoader{}.load(*node).config;
}

std::optional<passable_area::tools::FalseObstacleBagSummary> RunFalseObstacleReplay(
    const FalseObstacleReplayArgs &args) {
  if (!std::filesystem::exists(args.bag_path)) {
    std::cerr << "bag path does not exist: " << args.bag_path << '\n';
    return std::nullopt;
  }

  const auto config = LoadConfigFromParamsFile(args.params_file);
  if (!config) {
    return std::nullopt;
  }

  rosbag2_cpp::Reader reader;
  reader.open(args.bag_path);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  std::map<int64_t, sensor_msgs::msg::PointCloud2> clouds;
  std::map<int64_t, int64_t> cloud_bag_times;
  std::map<int64_t, nav_msgs::msg::Odometry> odoms;
  std::optional<int64_t> bag_start_time;
  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    if (!bag_start_time.has_value()) {
      bag_start_time = bag_msg->time_stamp;
    } else {
      bag_start_time = std::min(*bag_start_time, bag_msg->time_stamp);
    }
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    if (bag_msg->topic_name == "/LOC_BODY_POINTS") {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_ser.deserialize_message(&serialized, &cloud_msg);
      const int64_t stamp = rclcpp::Time(cloud_msg.header.stamp).nanoseconds();
      clouds.emplace(stamp, std::move(cloud_msg));
      cloud_bag_times.emplace(stamp, bag_msg->time_stamp);
    } else if (bag_msg->topic_name == "/ODOM") {
      nav_msgs::msg::Odometry odom_msg;
      odom_ser.deserialize_message(&serialized, &odom_msg);
      odoms.emplace(rclcpp::Time(odom_msg.header.stamp).nanoseconds(), std::move(odom_msg));
    }
  }

  passable_area::interfaces::ros::PointCloudConverter cloud_converter;
  passable_area::interfaces::ros::OdomConverter odom_converter;
  passable_area::core::Processor processor(*config);
  passable_area::tools::FalseObstacleAnalyzer analyzer(*config, args.analyzer_config);

  int total_frames = 0;
  int rear_dropout_frames = 0;
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
    if (output.observability.rear_dropout) {
      ++rear_dropout_frames;
    }
    const auto analysis = analyzer.analyzeFrame(output);
    if (analysis) {
      auto frame_analysis = *analysis;
      const auto bag_time_it = cloud_bag_times.find(stamp);
      if (bag_start_time.has_value() && bag_time_it != cloud_bag_times.end()) {
        frame_analysis.start_offset_sec =
            static_cast<double>(bag_time_it->second - *bag_start_time) * 1e-9;
      }
      candidate_frames.push_back(std::move(frame_analysis));
      ++current_candidate_run;
      longest_candidate_run = std::max(longest_candidate_run, current_candidate_run);
    } else {
      current_candidate_run = 0;
    }
  }

  return analyzer.buildSummary(total_frames, rear_dropout_frames, longest_candidate_run,
                               std::move(candidate_frames), args.top_k);
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
    PrintFalseObstacleSummary(*summary, replay_args->bag_path, replay_args->params_file,
                              replay_args->top_k, TerminalStyle{replay_args->use_color});
    rclcpp::shutdown();
    return 0;
  }

  if (HasFlag(args, "--benchmark-timing") || (args.size() >= 2 && args[0] == "--bag")) {
    const auto bag_path = ParseStringFlagValue(args, "--bag");
    if (!bag_path) {
      std::cerr << "timing benchmark requires --bag <path>\n";
      rclcpp::shutdown();
      return 1;
    }
    const std::string params_file =
        ParseStringFlagValue(args, "--params-file").value_or(DefaultParamsFile());
    const auto config = LoadConfigFromParamsFile(params_file);
    if (!config) {
      rclcpp::shutdown();
      return 1;
    }
    const auto summary = RunBagReplay(*bag_path, *config);
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
              << " avg_input_points=" << std::setprecision(1) << summary->avg_input_point_count
              << " max_input_points=" << std::setprecision(0) << summary->max_input_point_count
              << " avg_processing_ms=" << std::setprecision(3) << summary->avg_processing_ms
              << " max_processing_ms=" << summary->max_processing_ms
              << " p95_processing_ms=" << summary->p95_processing_ms
              << '\n';
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
