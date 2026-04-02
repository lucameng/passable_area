#include "passable_area/tools/false_obstacle_analyzer.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace passable_area::tools {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct BinStats {
  int count = 0;
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
};

float NormalizeAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0f * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0f * kPi;
  }
  return angle;
}

int RootCauseIndex(FalseObstacleRootCause cause) {
  return static_cast<int>(cause);
}

bool IsInsideDetectionBox(const FalseObstacleDetectionBox &box, float x, float y) {
  return x >= box.x_min && x <= box.x_max && y >= box.y_min && y <= box.y_max;
}

int CellIndex(const passable_area::core::FrameOutput &output, float odom_x, float odom_y) {
  const int col = static_cast<int>(std::floor((odom_x - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(std::floor((odom_y - output.origin.y()) / output.resolution));
  if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
    return -1;
  }
  return row * output.cols + col;
}

passable_area::core::Point3f TransformBaseGravityPointToOdom(float x, float y,
                                                             const passable_area::core::Pose3D &pose) {
  const float yaw = passable_area::core::YawFromQuaternion(pose.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  return passable_area::core::Point3f{
      pose.position.x() + cos_yaw * x - sin_yaw * y,
      pose.position.y() + sin_yaw * x + cos_yaw * y,
      pose.position.z()};
}

int SectorIndexForBaseGravityPoint(const passable_area::core::FrameOutput &output, float x, float y) {
  if (output.observability.sectors.empty()) {
    return -1;
  }
  const float angle = NormalizeAngle(std::atan2(y, x));
  const float angle_per_sector =
      2.0f * kPi / static_cast<float>(std::max<size_t>(output.observability.sectors.size(), 1U));
  return std::clamp(static_cast<int>(std::floor((angle + kPi) / angle_per_sector)), 0,
                    static_cast<int>(output.observability.sectors.size()) - 1);
}

int64_t MakeBinKey(int x_bin, int y_bin) {
  return (static_cast<int64_t>(x_bin) << 32) ^
         static_cast<uint32_t>(y_bin);
}

} // namespace

FalseObstacleAnalyzer::FalseObstacleAnalyzer(const passable_area::core::Config &config,
                                             const FalseObstacleAnalyzerConfig &analysis_config)
    : config_(config), analysis_config_(analysis_config) {}

std::optional<FalseObstacleFrameAnalysis> FalseObstacleAnalyzer::analyzeFrame(
    const passable_area::core::FrameOutput &output) const {
  std::unordered_map<int64_t, BinStats> bins;
  int in_box_count = 0;
  for (const auto &obstacle_point : output.obstacle_points) {
    const float x = obstacle_point.point.x;
    const float y = obstacle_point.point.y;
    if (!IsInsideDetectionBox(analysis_config_.detection_box, x, y)) {
      continue;
    }
    ++in_box_count;
    const int x_bin = static_cast<int>(std::floor((x - analysis_config_.detection_box.x_min) /
                                                  analysis_config_.hotspot_bin_size));
    const int y_bin = static_cast<int>(std::floor((y - analysis_config_.detection_box.y_min) /
                                                  analysis_config_.hotspot_bin_size));
    auto &bin = bins[MakeBinKey(x_bin, y_bin)];
    ++bin.count;
    bin.sum_x += x;
    bin.sum_y += y;
    bin.min_z = std::min(bin.min_z, obstacle_point.point.z);
    bin.max_z = std::max(bin.max_z, obstacle_point.point.z);
  }

  if (in_box_count == 0) {
    return std::nullopt;
  }

  FalseObstacleFrameAnalysis analysis;
  analysis.stamp = output.stamp;
  analysis.in_box_obstacle_point_count = in_box_count;
  analysis.frame_partial = output.observability.frame_partial;
  analysis.rear_dropout = output.observability.rear_dropout;
  analysis.max_local_obstacle_evidence = 0.0f;
  analysis.min_local_clearance = std::numeric_limits<float>::infinity();
  analysis.min_local_support_continuity = std::numeric_limits<float>::infinity();

  for (const auto &[key, bin] : bins) {
    (void)key;
    const float center_x = bin.sum_x / static_cast<float>(std::max(bin.count, 1));
    const float center_y = bin.sum_y / static_cast<float>(std::max(bin.count, 1));
    analysis.hotspots.push_back(
        buildHotspot(output, center_x, center_y, bin.min_z, bin.max_z, bin.count));
  }

  std::sort(analysis.hotspots.begin(), analysis.hotspots.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.severity > rhs.severity; });
  if (analysis.hotspots.size() > 3U) {
    analysis.hotspots.resize(3U);
  }

  float hotspot_severity_sum = 0.0f;
  for (const auto &hotspot : analysis.hotspots) {
    hotspot_severity_sum += hotspot.severity;
    analysis.max_local_obstacle_evidence =
        std::max(analysis.max_local_obstacle_evidence, hotspot.obstacle_evidence);
    if (std::isfinite(hotspot.clearance)) {
      analysis.min_local_clearance = std::min(analysis.min_local_clearance, hotspot.clearance);
    }
    if (std::isfinite(hotspot.support_continuity)) {
      analysis.min_local_support_continuity =
          std::min(analysis.min_local_support_continuity, hotspot.support_continuity);
    }
  }

  if (!std::isfinite(analysis.min_local_clearance)) {
    analysis.min_local_clearance = std::numeric_limits<float>::quiet_NaN();
  }
  if (!std::isfinite(analysis.min_local_support_continuity)) {
    analysis.min_local_support_continuity = std::numeric_limits<float>::quiet_NaN();
  }

  analysis.severity = static_cast<float>(analysis.in_box_obstacle_point_count) + hotspot_severity_sum;
  analysis.classification = reduceFrameClass(analysis.hotspots);
  return analysis;
}

FalseObstacleBagSummary FalseObstacleAnalyzer::buildSummary(
    int total_frames, int rear_dropout_frames, int longest_consecutive_candidate_run,
    std::vector<FalseObstacleFrameAnalysis> candidate_frames, int top_k) const {
  const int total_candidate_frames = static_cast<int>(candidate_frames.size());
  std::array<int, 5> root_cause_counts = {0, 0, 0, 0, 0};
  for (const auto &frame : candidate_frames) {
    ++root_cause_counts[RootCauseIndex(frame.classification)];
  }

  std::sort(candidate_frames.begin(), candidate_frames.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.severity > rhs.severity; });
  if (top_k >= 0 && static_cast<int>(candidate_frames.size()) > top_k) {
    candidate_frames.resize(static_cast<size_t>(top_k));
  }

  FalseObstacleBagSummary summary;
  summary.detection_box = analysis_config_.detection_box;
  summary.total_frames = total_frames;
  summary.candidate_frames = total_candidate_frames;
  summary.rear_dropout_frames = rear_dropout_frames;
  summary.longest_consecutive_candidate_run = longest_consecutive_candidate_run;
  summary.root_cause_counts = root_cause_counts;
  summary.ranked_frames = std::move(candidate_frames);
  return summary;
}

FalseObstacleAnalyzer::LocalCellContext FalseObstacleAnalyzer::lookupLocalContext(
    const passable_area::core::FrameOutput &output, float x, float y) const {
  LocalCellContext context;
  const auto point_in_odom = TransformBaseGravityPointToOdom(x, y, output.base_pose_in_odom);
  const int cell = CellIndex(output, point_in_odom.x, point_in_odom.y);
  if (cell >= 0) {
    context.has_grid_values = true;
    context.obstacle_evidence = output.obstacle_evidence[static_cast<size_t>(cell)];
    context.clearance = output.clearance[static_cast<size_t>(cell)];
    context.support_continuity = output.support_continuity[static_cast<size_t>(cell)];
    context.overhead_height = output.overhead_height[static_cast<size_t>(cell)];
  }

  const int sector_index = SectorIndexForBaseGravityPoint(output, x, y);
  if (sector_index >= 0) {
    context.has_observability = true;
    context.observability_state = output.observability.sectors[static_cast<size_t>(sector_index)].state;
  }
  return context;
}

FalseObstacleHotspot FalseObstacleAnalyzer::buildHotspot(
    const passable_area::core::FrameOutput &output, float x, float y, float min_z, float max_z,
    int obstacle_point_count) const {
  FalseObstacleHotspot hotspot;
  hotspot.x = x;
  hotspot.y = y;
  hotspot.min_z = min_z;
  hotspot.max_z = max_z;
  hotspot.obstacle_point_count = obstacle_point_count;

  const auto context = lookupLocalContext(output, x, y);
  hotspot.has_grid_values = context.has_grid_values;
  hotspot.obstacle_evidence = context.obstacle_evidence;
  hotspot.clearance = context.clearance;
  hotspot.support_continuity = context.support_continuity;
  hotspot.overhead_height = context.overhead_height;
  hotspot.has_observability = context.has_observability;
  hotspot.observability_state = context.observability_state;
  hotspot.classification = classifyHotspot(hotspot);
  hotspot.severity = computeHotspotSeverity(hotspot);

  switch (hotspot.classification) {
    case FalseObstacleRootCause::kClearanceDriven:
      hotspot.explanation = "clearance below threshold";
      break;
    case FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity:
      hotspot.explanation = "high obstacle evidence + low continuity";
      break;
    case FalseObstacleRootCause::kObstacleEvidenceDriven:
      hotspot.explanation = "high obstacle evidence";
      break;
    case FalseObstacleRootCause::kObservabilityInfluenced:
      hotspot.explanation = "observability degraded, direct trigger unclear";
      break;
    case FalseObstacleRootCause::kUnknownOrMixed:
      hotspot.explanation = "mixed or insufficient local evidence";
      break;
  }
  return hotspot;
}

float FalseObstacleAnalyzer::computeHotspotSeverity(const FalseObstacleHotspot &hotspot) const {
  const float evidence_score = std::clamp(hotspot.obstacle_evidence, 0.0f, 1.0f);
  const float continuity_penalty =
      std::isfinite(hotspot.support_continuity)
          ? std::clamp(analysis_config_.support_continuity_low_threshold - hotspot.support_continuity,
                       0.0f, analysis_config_.support_continuity_low_threshold) /
                std::max(analysis_config_.support_continuity_low_threshold, 1e-3f)
          : 0.0f;
  const float clearance_penalty =
      (std::isfinite(hotspot.clearance) && hotspot.clearance < config_.geometry.min_clearance) ? 1.0f
                                                                                                : 0.0f;
  float observability_penalty = 0.0f;
  if (hotspot.has_observability) {
    if (hotspot.observability_state == passable_area::core::ObservabilityState::kPartiallyObserved) {
      observability_penalty = 0.5f;
    } else if (hotspot.observability_state ==
               passable_area::core::ObservabilityState::kMissingByDropout) {
      observability_penalty = 1.0f;
    }
  }

  return static_cast<float>(hotspot.obstacle_point_count) + 2.0f * evidence_score +
         2.0f * clearance_penalty + continuity_penalty + 0.5f * observability_penalty;
}

FalseObstacleRootCause FalseObstacleAnalyzer::classifyHotspot(
    const FalseObstacleHotspot &hotspot) const {
  if (hotspot.has_grid_values && std::isfinite(hotspot.clearance) &&
      hotspot.clearance < config_.geometry.min_clearance &&
      // v1 approximation: finite overhead_height is the currently available proxy
      // that the low-clearance path is backed by an upper-structure context.
      std::isfinite(hotspot.overhead_height)) {
    return FalseObstacleRootCause::kClearanceDriven;
  }
  if (hotspot.has_grid_values &&
      hotspot.obstacle_evidence >= analysis_config_.obstacle_evidence_high_threshold &&
      std::isfinite(hotspot.support_continuity) &&
      hotspot.support_continuity < analysis_config_.support_continuity_low_threshold) {
    return FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity;
  }
  if (hotspot.has_grid_values &&
      hotspot.obstacle_evidence >= analysis_config_.obstacle_evidence_high_threshold) {
    return FalseObstacleRootCause::kObstacleEvidenceDriven;
  }
  if (hotspot.has_observability &&
      hotspot.observability_state != passable_area::core::ObservabilityState::kObserved) {
    return FalseObstacleRootCause::kObservabilityInfluenced;
  }
  return FalseObstacleRootCause::kUnknownOrMixed;
}

FalseObstacleRootCause FalseObstacleAnalyzer::reduceFrameClass(
    const std::vector<FalseObstacleHotspot> &hotspots) const {
  if (hotspots.empty()) {
    return FalseObstacleRootCause::kUnknownOrMixed;
  }
  if (hotspots.size() >= 2U && hotspots[0].classification != hotspots[1].classification) {
    const float severity_gap = std::abs(hotspots[0].severity - hotspots[1].severity);
    if (severity_gap <= analysis_config_.mixed_severity_ratio * std::max(hotspots[0].severity, 1e-3f)) {
      return FalseObstacleRootCause::kUnknownOrMixed;
    }
  }
  return hotspots.front().classification;
}

const char *ToString(FalseObstacleRootCause cause) {
  switch (cause) {
    case FalseObstacleRootCause::kClearanceDriven:
      return "ClearanceDriven";
    case FalseObstacleRootCause::kObstacleEvidenceDriven:
      return "ObstacleEvidenceDriven";
    case FalseObstacleRootCause::kObstacleEvidencePlusLowContinuity:
      return "ObstacleEvidencePlusLowContinuity";
    case FalseObstacleRootCause::kObservabilityInfluenced:
      return "ObservabilityInfluenced";
    case FalseObstacleRootCause::kUnknownOrMixed:
      return "UnknownOrMixed";
  }
  return "UnknownOrMixed";
}

const char *ToString(passable_area::core::ObservabilityState state) {
  switch (state) {
    case passable_area::core::ObservabilityState::kObserved:
      return "Observed";
    case passable_area::core::ObservabilityState::kPartiallyObserved:
      return "PartiallyObserved";
    case passable_area::core::ObservabilityState::kMissingByDropout:
      return "MissingByDropout";
  }
  return "Unknown";
}

} // namespace passable_area::tools
