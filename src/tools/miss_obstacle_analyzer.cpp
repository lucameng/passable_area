#include "passable_area/tools/miss_obstacle_analyzer.hpp"

#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace passable_area::tools {
namespace {

struct RoiSampleStats {
  int sample_count = 0;
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  float min_relative_z = std::numeric_limits<float>::infinity();
  float max_relative_z = -std::numeric_limits<float>::infinity();
};

bool IsInsideDetectionBox(const MissObstacleDetectionBox &box, float x, float y) {
  return x >= box.x_min && x <= box.x_max && y >= box.y_min && y <= box.y_max;
}

passable_area::core::Point3f TransformOdomPointToBaseGravity(float x, float y, float z,
                                                             const passable_area::core::Pose3D &pose) {
  const float yaw = passable_area::core::YawFromQuaternion(pose.orientation);
  const float cos_yaw = std::cos(yaw);
  const float sin_yaw = std::sin(yaw);
  const float dx = x - pose.position.x();
  const float dy = y - pose.position.y();
  return passable_area::core::Point3f{cos_yaw * dx + sin_yaw * dy,
                                      -sin_yaw * dx + cos_yaw * dy,
                                      z - pose.position.z()};
}

int CellIndex(const passable_area::core::FrameOutput &output, float odom_x, float odom_y) {
  const int col = static_cast<int>(std::floor((odom_x - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(std::floor((odom_y - output.origin.y()) / output.resolution));
  if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
    return -1;
  }
  return row * output.cols + col;
}

int RootCauseIndex(MissObstacleRootCause cause) {
  return static_cast<int>(cause);
}

} // namespace

MissObstacleAnalyzer::MissObstacleAnalyzer(const passable_area::core::Config &config,
                                           const MissObstacleAnalyzerConfig &analysis_config)
    : config_(config), analysis_config_(analysis_config) {}

std::optional<MissObstacleFrameAnalysis> MissObstacleAnalyzer::analyzeFrame(
    const passable_area::core::FrameOutput &output,
    const passable_area::core::ProcessedFrame &processed_frame) const {
  std::unordered_map<int, RoiSampleStats> sample_stats_by_cell;
  sample_stats_by_cell.reserve(processed_frame.odom_samples.size() / 8U + 1U);

  int roi_sample_count = 0;
  int roi_obstacle_point_count = 0;
  std::unordered_set<int> roi_obstacle_point_source_cells;
  roi_obstacle_point_source_cells.reserve(output.obstacle_points.size());

  for (const auto &sample : processed_frame.odom_samples) {
    const auto point_in_base_gravity = TransformOdomPointToBaseGravity(
        sample.point_in_odom.x, sample.point_in_odom.y, sample.point_in_odom.z, output.base_pose_in_odom);
    if (!IsInsideDetectionBox(analysis_config_.detection_box, point_in_base_gravity.x,
                              point_in_base_gravity.y)) {
      continue;
    }
    ++roi_sample_count;
    const int cell = CellIndex(output, sample.point_in_odom.x, sample.point_in_odom.y);
    if (cell < 0) {
      continue;
    }
    auto &stats = sample_stats_by_cell[cell];
    ++stats.sample_count;
    stats.min_z = std::min(stats.min_z, sample.point_in_odom.z);
    stats.max_z = std::max(stats.max_z, sample.point_in_odom.z);
    stats.min_relative_z = std::min(stats.min_relative_z, point_in_base_gravity.z);
    stats.max_relative_z = std::max(stats.max_relative_z, point_in_base_gravity.z);
  }

  for (const auto &obstacle_point : output.obstacle_points) {
    if (!IsInsideDetectionBox(analysis_config_.detection_box, obstacle_point.point.x,
                              obstacle_point.point.y)) {
      continue;
    }
    ++roi_obstacle_point_count;
    if (obstacle_point.source_cell >= 0) {
      roi_obstacle_point_source_cells.insert(obstacle_point.source_cell);
    }
  }

  if (roi_obstacle_point_count > 0) {
    return std::nullopt;
  }

  MissObstacleFrameAnalysis analysis;
  analysis.stamp = output.stamp;
  analysis.roi_sample_count = roi_sample_count;
  analysis.roi_cells_with_any_samples = static_cast<int>(sample_stats_by_cell.size());
  analysis.roi_obstacle_point_count = 0;
  analysis.min_clearance = std::numeric_limits<float>::infinity();
  analysis.min_support_continuity = std::numeric_limits<float>::infinity();

  int strong_evidence_cell_count = 0;
  int strong_evidence_cells_with_samples = 0;
  int publishable_sample_count = 0;
  std::vector<std::pair<float, MissObstacleRepresentativeCell>> ranked_cells;
  ranked_cells.reserve(sample_stats_by_cell.size() + 8U);

  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const float odom_x =
          output.origin.x() + (static_cast<float>(col) + 0.5f) * output.resolution;
      const float odom_y =
          output.origin.y() + (static_cast<float>(row) + 0.5f) * output.resolution;
      const auto point_in_base_gravity =
          TransformOdomPointToBaseGravity(odom_x, odom_y, output.support_height[row * output.cols + col],
                                          output.base_pose_in_odom);
      if (!IsInsideDetectionBox(analysis_config_.detection_box, point_in_base_gravity.x,
                                point_in_base_gravity.y)) {
        continue;
      }

      const int idx = row * output.cols + col;
      const auto sample_it = sample_stats_by_cell.find(idx);
      const bool has_samples = sample_it != sample_stats_by_cell.end();
      const RoiSampleStats sample_stats = has_samples ? sample_it->second : RoiSampleStats{};

      if (output.obstacle_suspicious[idx] != 0U) {
        ++analysis.obstacle_suspicious_cell_count;
      }
      if (output.obstacle_candidate_cell[idx] != 0U) {
        ++analysis.obstacle_candidate_cell_count;
      }
      if (output.obstacle_rejected_by_neighbor_support[idx] != 0U) {
        ++analysis.rejected_suspicious_cell_count;
      }
      analysis.max_obstacle_evidence =
          std::max(analysis.max_obstacle_evidence, output.obstacle_evidence[idx]);
      analysis.max_support_confidence =
          std::max(analysis.max_support_confidence, output.support_confidence[idx]);
      if (std::isfinite(output.clearance[idx])) {
        analysis.min_clearance = std::min(analysis.min_clearance, output.clearance[idx]);
      }
      if (std::isfinite(output.support_continuity[idx])) {
        analysis.min_support_continuity =
            std::min(analysis.min_support_continuity, output.support_continuity[idx]);
      }

      float support_ref = output.support_height[idx];
      if (!std::isfinite(support_ref) && has_samples && sample_stats.sample_count > 0) {
        support_ref = sample_stats.min_z;
      }

      if (output.obstacle_evidence[idx] >= config_.obstacle_points_min_evidence) {
        ++strong_evidence_cell_count;
        if (has_samples && sample_stats.sample_count > 0) {
          ++strong_evidence_cells_with_samples;
        }
      }

      if (has_samples && sample_stats.sample_count > 0 && std::isfinite(support_ref) &&
          output.obstacle_evidence[idx] >= config_.obstacle_points_min_evidence &&
          sample_stats.max_z >= support_ref + config_.obstacle_points_min_height) {
        publishable_sample_count += sample_stats.sample_count;
      }

      const bool interesting = has_samples || output.obstacle_suspicious[idx] != 0U ||
                               output.obstacle_candidate_cell[idx] != 0U ||
                               output.obstacle_rejected_by_neighbor_support[idx] != 0U ||
                               output.upper_support_cell[idx] != 0U ||
                               output.obstacle_evidence[idx] > 0.0f ||
                               std::isfinite(output.overhead_height[idx]);
      if (!interesting) {
        continue;
      }

      MissObstacleRepresentativeCell cell;
      cell.x = point_in_base_gravity.x;
      cell.y = point_in_base_gravity.y;
      cell.sample_count = has_samples ? sample_stats.sample_count : 0;
      cell.min_sample_relative_z = has_samples ? sample_stats.min_relative_z
                                               : std::numeric_limits<float>::quiet_NaN();
      cell.max_sample_relative_z = has_samples ? sample_stats.max_relative_z
                                               : std::numeric_limits<float>::quiet_NaN();
      cell.support_height = output.support_height[idx];
      cell.support_ref = support_ref;
      cell.overhead_height = output.overhead_height[idx];
      cell.obstacle_evidence = output.obstacle_evidence[idx];
      cell.support_confidence = output.support_confidence[idx];
      cell.support_anchor_used = output.support_anchor_used[idx];
      cell.sub_support_leak_count = output.sub_support_leak_count[idx];
      cell.upper_support_cell = output.upper_support_cell[idx] != 0U;
      cell.obstacle_suspicious = output.obstacle_suspicious[idx] != 0U;
      cell.obstacle_candidate_cell = output.obstacle_candidate_cell[idx] != 0U;
      cell.obstacle_rejected_by_neighbor_support =
          output.obstacle_rejected_by_neighbor_support[idx] != 0U;
      cell.neighbor_upper_support_count = output.neighbor_upper_support_count[idx];
      cell.max_sample_z_minus_support_ref =
          (has_samples && sample_stats.sample_count > 0 && std::isfinite(support_ref))
              ? sample_stats.max_z - support_ref
              : std::numeric_limits<float>::quiet_NaN();

      if (cell.obstacle_rejected_by_neighbor_support) {
        cell.explanation = "suspicious obstacle rejected by neighborhood gate";
      } else if (cell.obstacle_candidate_cell &&
                 cell.obstacle_evidence < config_.obstacle_points_min_evidence) {
        cell.explanation = "frontend candidate formed but obstacle evidence is still below publish threshold";
      } else if (output.obstacle_evidence[idx] >= config_.obstacle_points_min_evidence &&
                 (!std::isfinite(cell.max_sample_z_minus_support_ref) ||
                  cell.max_sample_z_minus_support_ref < config_.obstacle_points_min_height)) {
        cell.explanation = "obstacle evidence is high enough but no sample clears publish height gate";
      } else if (cell.sub_support_leak_count > 0U && !cell.obstacle_candidate_cell &&
                 !cell.obstacle_rejected_by_neighbor_support) {
        cell.explanation = "support anchor filtered lower-layer leak samples before obstacle promotion";
      } else if (!cell.obstacle_suspicious) {
        cell.explanation = "samples did not create enough vertical separation to become suspicious";
      } else {
        cell.explanation = "mixed local evidence";
      }

      float score = static_cast<float>(cell.sample_count);
      score += cell.obstacle_rejected_by_neighbor_support ? 4.0f : 0.0f;
      score += cell.obstacle_candidate_cell ? 3.0f : 0.0f;
      score += cell.obstacle_suspicious ? 2.0f : 0.0f;
      score += cell.upper_support_cell ? 1.0f : 0.0f;
      score += 4.0f * std::clamp(cell.obstacle_evidence, 0.0f, 1.0f);
      ranked_cells.push_back({score, cell});
    }
  }

  if (!std::isfinite(analysis.min_clearance)) {
    analysis.min_clearance = std::numeric_limits<float>::quiet_NaN();
  }
  if (!std::isfinite(analysis.min_support_continuity)) {
    analysis.min_support_continuity = std::numeric_limits<float>::quiet_NaN();
  }

  if (analysis.roi_sample_count == 0) {
    analysis.classification = MissObstacleRootCause::kNoSamplesInRoi;
    analysis.explanation = "roi contains no odom samples, so obstacle points cannot be published";
    analysis.evidence_lines.push_back("roi_sample_count=0");
  } else if (analysis.rejected_suspicious_cell_count > 0 &&
             analysis.obstacle_candidate_cell_count == 0) {
    analysis.classification = MissObstacleRootCause::kRejectedByNeighborSupport;
    analysis.explanation =
        "roi contains suspicious obstacle cells, but all were rejected by the neighbor-support gate";
    analysis.evidence_lines.push_back(
        "rejected_suspicious_cells=" + std::to_string(analysis.rejected_suspicious_cell_count));
    analysis.evidence_lines.push_back(
        "obstacle_candidate_cells=" + std::to_string(analysis.obstacle_candidate_cell_count));
  } else if (strong_evidence_cell_count > 0 && publishable_sample_count == 0) {
    if (strong_evidence_cells_with_samples == 0) {
      analysis.classification = MissObstacleRootCause::kNoObstacleSourceSamplesInRoi;
      analysis.explanation =
          "some roi cells already have strong obstacle evidence, but no current roi samples land in those publishable cells";
      analysis.evidence_lines.push_back("strong_evidence_cells=" +
                                        std::to_string(strong_evidence_cell_count));
      analysis.evidence_lines.push_back("strong_evidence_cells_with_samples=" +
                                        std::to_string(strong_evidence_cells_with_samples));
    } else {
      analysis.classification = MissObstacleRootCause::kOutputHeightGateNotMet;
      analysis.explanation =
          "roi cells have enough obstacle evidence, but no current sample is high enough above support_ref to publish obstacle points";
      analysis.evidence_lines.push_back("strong_evidence_cells=" +
                                        std::to_string(strong_evidence_cell_count));
      analysis.evidence_lines.push_back("obstacle_points_min_height=" +
                                        std::to_string(config_.obstacle_points_min_height));
    }
  } else if (analysis.obstacle_suspicious_cell_count == 0 &&
             analysis.obstacle_candidate_cell_count == 0) {
    uint32_t leak_total = 0U;
    for (const auto &[cell, _] : sample_stats_by_cell) {
      leak_total += output.sub_support_leak_count[static_cast<size_t>(cell)];
    }
    if (leak_total > 0U) {
      analysis.classification = MissObstacleRootCause::kLeakFilteredToNoCandidate;
      analysis.explanation =
          "support-anchor leak suppression removed lower-layer samples before obstacle candidates formed";
      analysis.evidence_lines.push_back("sub_support_leak_count_total=" + std::to_string(leak_total));
    } else {
      analysis.classification = MissObstacleRootCause::kNoFrontendObstacleSuspicion;
      analysis.explanation =
          "roi has samples, but no cell reached the frontend suspicious-obstacle trigger";
      analysis.evidence_lines.push_back(
          "obstacle_suspicious_cells=" + std::to_string(analysis.obstacle_suspicious_cell_count));
    }
  } else if (analysis.max_obstacle_evidence < config_.obstacle_points_min_evidence) {
    analysis.classification = MissObstacleRootCause::kObstacleEvidenceTooLow;
    analysis.explanation =
        "frontend obstacle evidence exists, but it never reached the publish threshold for obstacle points";
    analysis.evidence_lines.push_back("max_obstacle_evidence=" +
                                      std::to_string(analysis.max_obstacle_evidence));
    analysis.evidence_lines.push_back("obstacle_points_min_evidence=" +
                                      std::to_string(config_.obstacle_points_min_evidence));
  } else {
    analysis.classification = MissObstacleRootCause::kUnknownOrMixed;
    analysis.explanation = "mixed evidence: no obstacle points were published, but no single blocking stage dominates";
    analysis.evidence_lines.push_back(
        "obstacle_candidate_cells=" + std::to_string(analysis.obstacle_candidate_cell_count));
    analysis.evidence_lines.push_back("max_obstacle_evidence=" +
                                      std::to_string(analysis.max_obstacle_evidence));
  }

  analysis.evidence_lines.push_back("roi_cells_with_any_samples=" +
                                    std::to_string(analysis.roi_cells_with_any_samples));
  analysis.evidence_lines.push_back("roi_obstacle_point_sources=" +
                                    std::to_string(roi_obstacle_point_source_cells.size()));

  std::sort(ranked_cells.begin(), ranked_cells.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.first > rhs.first; });
  const int rep_limit = std::max(1, analysis_config_.representative_cell_limit);
  for (int i = 0; i < static_cast<int>(ranked_cells.size()) && i < rep_limit; ++i) {
    analysis.representative_cells.push_back(ranked_cells[static_cast<size_t>(i)].second);
  }

  analysis.severity = static_cast<float>(analysis.roi_sample_count) +
                      2.0f * static_cast<float>(analysis.obstacle_suspicious_cell_count) +
                      3.0f * static_cast<float>(analysis.obstacle_candidate_cell_count) +
                      4.0f * static_cast<float>(analysis.rejected_suspicious_cell_count) +
                      4.0f * std::clamp(analysis.max_obstacle_evidence, 0.0f, 1.0f);
  return analysis;
}

MissObstacleBagSummary MissObstacleAnalyzer::buildSummary(
    int total_frames, std::vector<MissObstacleFrameAnalysis> candidate_frames) const {
  MissObstacleBagSummary summary;
  summary.detection_box = analysis_config_.detection_box;
  summary.total_frames = total_frames;
  summary.missed_frames = static_cast<int>(candidate_frames.size());
  for (const auto &frame : candidate_frames) {
    ++summary.root_cause_counts[RootCauseIndex(frame.classification)];
  }
  std::sort(candidate_frames.begin(), candidate_frames.end(),
            [](const auto &lhs, const auto &rhs) { return lhs.severity > rhs.severity; });
  summary.ranked_frames = std::move(candidate_frames);
  return summary;
}

const char *ToString(MissObstacleRootCause cause) {
  switch (cause) {
    case MissObstacleRootCause::kNoSamplesInRoi:
      return "NoSamplesInRoi";
    case MissObstacleRootCause::kNoFrontendObstacleSuspicion:
      return "NoFrontendObstacleSuspicion";
    case MissObstacleRootCause::kRejectedByNeighborSupport:
      return "RejectedByNeighborSupport";
    case MissObstacleRootCause::kLeakFilteredToNoCandidate:
      return "LeakFilteredToNoCandidate";
    case MissObstacleRootCause::kObstacleEvidenceTooLow:
      return "ObstacleEvidenceTooLow";
    case MissObstacleRootCause::kOutputHeightGateNotMet:
      return "OutputHeightGateNotMet";
    case MissObstacleRootCause::kNoObstacleSourceSamplesInRoi:
      return "NoObstacleSourceSamplesInRoi";
    case MissObstacleRootCause::kUnknownOrMixed:
      return "UnknownOrMixed";
  }
  return "UnknownOrMixed";
}

} // namespace passable_area::tools
