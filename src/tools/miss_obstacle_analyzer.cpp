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
  float min_base_link_z = std::numeric_limits<float>::infinity();
  float max_base_link_z = -std::numeric_limits<float>::infinity();
};

bool IsInsideDetectionBox(const MissObstacleDetectionBox &box, float x,
                          float y) {
  return x >= box.x_min && x <= box.x_max && y >= box.y_min && y <= box.y_max;
}

bool PassesObstaclePointPublishHeightGates(
    const passable_area::core::Config &config, float support_ref,
    const RoiSampleStats &sample_stats) {
  return std::isfinite(support_ref) &&
         sample_stats.max_z >=
             support_ref + config.obstacle_points_min_height &&
         sample_stats.max_z > support_ref + config.geometry.max_step_up &&
         sample_stats.min_base_link_z <=
             config.obstacle_points_max_height_in_base_link;
}

passable_area::core::Point3f
TransformMapPointToBaseGravity(float x, float y, float z,
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

int CellIndex(const passable_area::core::FrameOutput &output, float map_x,
              float map_y) {
  const int col = static_cast<int>(
      std::floor((map_x - output.origin.x()) / output.resolution));
  const int row = static_cast<int>(
      std::floor((map_y - output.origin.y()) / output.resolution));
  if (row < 0 || row >= output.rows || col < 0 || col >= output.cols) {
    return -1;
  }
  return row * output.cols + col;
}

int RootCauseIndex(MissObstacleRootCause cause) {
  return static_cast<int>(cause);
}

} // namespace

MissObstacleAnalyzer::MissObstacleAnalyzer(
    const passable_area::core::Config &config,
    const MissObstacleAnalyzerConfig &analysis_config)
    : config_(config), analysis_config_(analysis_config) {}

std::optional<MissObstacleFrameAnalysis> MissObstacleAnalyzer::analyzeFrame(
    const passable_area::core::FrameOutput &output,
    const passable_area::core::ProcessedFrame &processed_frame) const {
  std::unordered_map<int, RoiSampleStats> sample_stats_by_cell;
  sample_stats_by_cell.reserve(processed_frame.map_samples.size() / 8U + 1U);

  int roi_sample_count = 0;
  int roi_obstacle_point_count = 0;
  std::unordered_set<int> roi_obstacle_point_source_cells;
  roi_obstacle_point_source_cells.reserve(output.obstacle_points.size());

  for (const auto &sample : processed_frame.map_samples) {
    const auto point_in_base_gravity = TransformMapPointToBaseGravity(
        sample.point_in_map.x, sample.point_in_map.y, sample.point_in_map.z,
        output.base_pose_in_map);
    if (!IsInsideDetectionBox(analysis_config_.detection_box,
                              point_in_base_gravity.x,
                              point_in_base_gravity.y)) {
      continue;
    }
    ++roi_sample_count;
    const int cell =
        CellIndex(output, sample.point_in_map.x, sample.point_in_map.y);
    if (cell < 0) {
      continue;
    }
    auto &stats = sample_stats_by_cell[cell];
    ++stats.sample_count;
    stats.min_z = std::min(stats.min_z, sample.point_in_map.z);
    stats.max_z = std::max(stats.max_z, sample.point_in_map.z);
    stats.min_relative_z =
        std::min(stats.min_relative_z, point_in_base_gravity.z);
    stats.max_relative_z =
        std::max(stats.max_relative_z, point_in_base_gravity.z);
    stats.min_base_link_z =
        std::min(stats.min_base_link_z, sample.point_in_base.z);
    stats.max_base_link_z =
        std::max(stats.max_base_link_z, sample.point_in_base.z);
  }

  for (const auto &obstacle_point : output.obstacle_points) {
    if (!IsInsideDetectionBox(analysis_config_.detection_box,
                              obstacle_point.point.x, obstacle_point.point.y)) {
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
  analysis.roi_cells_with_any_samples =
      static_cast<int>(sample_stats_by_cell.size());
  analysis.roi_obstacle_point_count = 0;
  analysis.min_clearance = std::numeric_limits<float>::infinity();
  analysis.min_support_continuity = std::numeric_limits<float>::infinity();

  int strong_evidence_cell_count = 0;
  int strong_evidence_cells_with_samples = 0;
  int publishable_sample_count = 0;
  int reasoner_blocked_cell_count = 0;
  int reasoner_candidate_not_blocked_count = 0;
  std::vector<std::pair<float, MissObstacleRepresentativeCell>> ranked_cells;
  ranked_cells.reserve(sample_stats_by_cell.size() + 8U);

  for (int row = 0; row < output.rows; ++row) {
    for (int col = 0; col < output.cols; ++col) {
      const float map_x = output.origin.x() +
                          (static_cast<float>(col) + 0.5f) * output.resolution;
      const float map_y = output.origin.y() +
                          (static_cast<float>(row) + 0.5f) * output.resolution;
      const auto point_in_base_gravity = TransformMapPointToBaseGravity(
          map_x, map_y, output.support_height[row * output.cols + col],
          output.base_pose_in_map);
      if (!IsInsideDetectionBox(analysis_config_.detection_box,
                                point_in_base_gravity.x,
                                point_in_base_gravity.y)) {
        continue;
      }

      const int idx = row * output.cols + col;
      const auto sample_it = sample_stats_by_cell.find(idx);
      const bool has_samples = sample_it != sample_stats_by_cell.end();
      const RoiSampleStats sample_stats =
          has_samples ? sample_it->second : RoiSampleStats{};

      if (output.obstacle_suspicious[idx] != 0U) {
        ++analysis.obstacle_suspicious_cell_count;
      }
      if (output.obstacle_candidate_cell[idx] != 0U) {
        ++analysis.obstacle_candidate_cell_count;
      }

      analysis.max_obstacle_evidence = std::max(analysis.max_obstacle_evidence,
                                                output.obstacle_evidence[idx]);
      const uint8_t block_reason =
          output.block_reason.empty() ? 0U : output.block_reason[idx];
      if (block_reason != 0U) {
        ++reasoner_blocked_cell_count;
      } else if (!output.block_reason.empty() &&
                 (output.obstacle_candidate_cell[idx] != 0U ||
                  output.obstacle_evidence[idx] > 0.0f)) {
        ++reasoner_candidate_not_blocked_count;
      }
      analysis.max_support_confidence = std::max(
          analysis.max_support_confidence, output.support_confidence[idx]);
      if (std::isfinite(output.clearance[idx])) {
        analysis.min_clearance =
            std::min(analysis.min_clearance, output.clearance[idx]);
      }
      if (std::isfinite(output.support_continuity[idx])) {
        analysis.min_support_continuity = std::min(
            analysis.min_support_continuity, output.support_continuity[idx]);
      }

      float support_ref = output.support_height[idx];

      if (output.obstacle_evidence[idx] >=
          config_.obstacle_points_min_evidence) {
        ++strong_evidence_cell_count;
        if (has_samples && sample_stats.sample_count > 0) {
          ++strong_evidence_cells_with_samples;
        }
      }

      if (has_samples && sample_stats.sample_count > 0 &&
          output.obstacle_evidence[idx] >=
              config_.obstacle_points_min_evidence &&
          PassesObstaclePointPublishHeightGates(config_, support_ref,
                                                sample_stats)) {
        publishable_sample_count += sample_stats.sample_count;
      }

      const bool interesting =
          has_samples || output.obstacle_suspicious[idx] != 0U ||
          output.obstacle_candidate_cell[idx] != 0U ||
          block_reason != 0U ||
          (!output.obstacle_point_publish_status.empty() &&
           output.obstacle_point_publish_status[idx] != 0U) ||
          output.obstacle_evidence[idx] > 0.0f ||
          std::isfinite(output.overhead_height[idx]);
      if (!interesting) {
        continue;
      }

      MissObstacleRepresentativeCell cell;
      cell.x = point_in_base_gravity.x;
      cell.y = point_in_base_gravity.y;
      cell.sample_count = has_samples ? sample_stats.sample_count : 0;
      cell.min_sample_relative_z =
          has_samples ? sample_stats.min_relative_z
                      : std::numeric_limits<float>::quiet_NaN();
      cell.max_sample_relative_z =
          has_samples ? sample_stats.max_relative_z
                      : std::numeric_limits<float>::quiet_NaN();
      cell.support_height = output.support_height[idx];
      cell.support_ref = support_ref;
      cell.overhead_height = output.overhead_height[idx];
      cell.protrusion_evidence = output.protrusion_evidence.empty()
                                     ? 0.0f
                                     : output.protrusion_evidence[idx];
      cell.overhead_evidence = output.overhead_evidence.empty()
                                   ? 0.0f
                                   : output.overhead_evidence[idx];
      cell.obstacle_evidence = output.obstacle_evidence[idx];
      cell.block_reason = block_reason;
      cell.obstacle_point_publish_status =
          output.obstacle_point_publish_status.empty()
              ? 0U
              : output.obstacle_point_publish_status[idx];
      cell.support_confidence = output.support_confidence[idx];
      cell.support_continuity = output.support_continuity[idx];
      cell.raw_sample_min_z = output.raw_sample_min_z.empty()
                                  ? std::numeric_limits<float>::quiet_NaN()
                                  : output.raw_sample_min_z[idx];
      cell.raw_sample_max_z = output.raw_sample_max_z.empty()
                                  ? std::numeric_limits<float>::quiet_NaN()
                                  : output.raw_sample_max_z[idx];
      cell.raw_sample_count =
          output.raw_sample_count.empty() ? 0U : output.raw_sample_count[idx];
      cell.filtered_sample_min_z = output.filtered_sample_min_z.empty()
                                       ? std::numeric_limits<float>::quiet_NaN()
                                       : output.filtered_sample_min_z[idx];
      cell.filtered_sample_max_z = output.filtered_sample_max_z.empty()
                                       ? std::numeric_limits<float>::quiet_NaN()
                                       : output.filtered_sample_max_z[idx];
      cell.filtered_sample_count = output.filtered_sample_count.empty()
                                       ? 0U
                                       : output.filtered_sample_count[idx];
      cell.obstacle_suspicious = output.obstacle_suspicious[idx] != 0U;
      cell.obstacle_candidate_cell = output.obstacle_candidate_cell[idx] != 0U;
      cell.max_sample_z_minus_support_ref =
          (has_samples && sample_stats.sample_count > 0 &&
           std::isfinite(support_ref))
              ? sample_stats.max_z - support_ref
              : std::numeric_limits<float>::quiet_NaN();

      if (!cell.obstacle_suspicious) {
        cell.explanation = "samples did not create enough vertical separation "
                           "to become suspicious";
      } else if (!cell.obstacle_candidate_cell) {
        cell.explanation =
            "cell became suspicious, but frontend did not retain an obstacle "
            "candidate";
      } else if (cell.block_reason == 0U) {
        cell.explanation =
            "frontend candidate exists, but reasoner did not mark the cell as "
            "blocked";
      } else if (cell.obstacle_evidence <
                 config_.obstacle_points_min_evidence) {
        cell.explanation =
            "candidate formed, but obstacle evidence is still below publish "
            "threshold";
      } else if (cell.obstacle_evidence >=
                     config_.obstacle_points_min_evidence &&
                 !PassesObstaclePointPublishHeightGates(config_, support_ref,
                                                        sample_stats)) {
        cell.explanation = "obstacle evidence is high enough but samples fail "
                           "the obstacle-point publish height gates";
      } else {
        cell.explanation = "mixed local evidence";
      }

      float score = static_cast<float>(cell.sample_count);
      score += cell.obstacle_candidate_cell ? 4.0f : 0.0f;
      score += cell.obstacle_suspicious ? 2.0f : 0.0f;
      score += block_reason != 0U ? 1.0f : 0.0f;
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
    analysis.explanation =
        "roi contains no odom samples, so obstacle points cannot be published";
    analysis.evidence_lines.push_back("roi_sample_count=0");
  } else if (reasoner_candidate_not_blocked_count > 0 &&
             reasoner_blocked_cell_count == 0) {
    analysis.classification = MissObstacleRootCause::kReasonerNotBlocked;
    analysis.explanation =
        "frontend or map evidence exists in the roi, but the reasoner did not "
        "mark any roi cell as blocked";
    analysis.evidence_lines.push_back(
        "reasoner_candidate_not_blocked_cells=" +
        std::to_string(reasoner_candidate_not_blocked_count));
  } else if (strong_evidence_cell_count > 0 && publishable_sample_count == 0) {
    if (strong_evidence_cells_with_samples == 0) {
      analysis.classification =
          MissObstacleRootCause::kNoObstacleSourceSamplesInRoi;
      analysis.explanation =
          "some roi cells already have strong obstacle evidence, but no "
          "current roi samples land in those publishable cells";
      analysis.evidence_lines.push_back(
          "strong_evidence_cells=" +
          std::to_string(strong_evidence_cell_count));
      analysis.evidence_lines.push_back(
          "strong_evidence_cells_with_samples=" +
          std::to_string(strong_evidence_cells_with_samples));
    } else {
      analysis.classification = MissObstacleRootCause::kOutputHeightGateNotMet;
      analysis.explanation =
          "roi cells have enough obstacle evidence, but no current sample "
          "passes the publish height gates in both base_gravity and base_link";
      analysis.evidence_lines.push_back(
          "strong_evidence_cells=" +
          std::to_string(strong_evidence_cell_count));
      analysis.evidence_lines.push_back(
          "obstacle_points_min_height=" +
          std::to_string(config_.obstacle_points_min_height));
      analysis.evidence_lines.push_back(
          "obstacle_points_max_height_in_base_link=" +
          std::to_string(config_.obstacle_points_max_height_in_base_link));
    }
  } else if (analysis.obstacle_suspicious_cell_count == 0 &&
             analysis.obstacle_candidate_cell_count == 0) {
    analysis.classification = MissObstacleRootCause::kNoFrontendObstacleSuspicion;
    analysis.explanation = "roi has samples, but no cell reached the frontend "
                           "suspicious-obstacle trigger";
    analysis.evidence_lines.push_back(
        "obstacle_suspicious_cells=" +
        std::to_string(analysis.obstacle_suspicious_cell_count));
  } else if (analysis.max_obstacle_evidence <
             config_.obstacle_points_min_evidence) {
    analysis.classification = MissObstacleRootCause::kObstacleEvidenceTooLow;
    analysis.explanation = "frontend obstacle evidence exists, but it never "
                           "reached the publish threshold for obstacle points";
    analysis.evidence_lines.push_back(
        "max_obstacle_evidence=" +
        std::to_string(analysis.max_obstacle_evidence));
    analysis.evidence_lines.push_back(
        "obstacle_points_min_evidence=" +
        std::to_string(config_.obstacle_points_min_evidence));
  } else {
    analysis.classification = MissObstacleRootCause::kUnknownOrMixed;
    analysis.explanation = "mixed evidence: no obstacle points were published, "
                           "but no single blocking stage dominates";
    analysis.evidence_lines.push_back(
        "obstacle_candidate_cells=" +
        std::to_string(analysis.obstacle_candidate_cell_count));
    analysis.evidence_lines.push_back(
        "max_obstacle_evidence=" +
        std::to_string(analysis.max_obstacle_evidence));
  }

  analysis.evidence_lines.push_back(
      "roi_cells_with_any_samples=" +
      std::to_string(analysis.roi_cells_with_any_samples));
  analysis.evidence_lines.push_back(
      "roi_obstacle_point_sources=" +
      std::to_string(roi_obstacle_point_source_cells.size()));

  std::sort(
      ranked_cells.begin(), ranked_cells.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.first > rhs.first; });
  const int rep_limit = std::max(1, analysis_config_.representative_cell_limit);
  for (int i = 0; i < static_cast<int>(ranked_cells.size()) && i < rep_limit;
       ++i) {
    analysis.representative_cells.push_back(
        ranked_cells[static_cast<size_t>(i)].second);
  }

  analysis.severity =
      static_cast<float>(analysis.roi_sample_count) +
      2.0f * static_cast<float>(analysis.obstacle_suspicious_cell_count) +
      3.0f * static_cast<float>(analysis.obstacle_candidate_cell_count) +
      4.0f * std::clamp(analysis.max_obstacle_evidence, 0.0f, 1.0f);
  return analysis;
}

MissObstacleBagSummary MissObstacleAnalyzer::buildSummary(
    int total_frames,
    std::vector<MissObstacleFrameAnalysis> candidate_frames) const {
  MissObstacleBagSummary summary;
  summary.detection_box = analysis_config_.detection_box;
  summary.total_frames = total_frames;
  summary.missed_frames = static_cast<int>(candidate_frames.size());
  for (const auto &frame : candidate_frames) {
    ++summary.root_cause_counts[RootCauseIndex(frame.classification)];
  }
  std::sort(candidate_frames.begin(), candidate_frames.end(),
            [](const auto &lhs, const auto &rhs) {
              return lhs.severity > rhs.severity;
            });
  summary.ranked_frames = std::move(candidate_frames);
  return summary;
}

const char *ToString(MissObstacleRootCause cause) {
  switch (cause) {
  case MissObstacleRootCause::kNoSamplesInRoi:
    return "NoSamplesInRoi";
  case MissObstacleRootCause::kNoFrontendCandidate:
    return "NoFrontendCandidate";
  case MissObstacleRootCause::kEvidenceTooLow:
    return "EvidenceTooLow";
  case MissObstacleRootCause::kPublishHeightGated:
    return "PublishHeightGated";
  case MissObstacleRootCause::kNoObstacleSourceSamplesInRoi:
    return "NoObstacleSourceSamplesInRoi";
  case MissObstacleRootCause::kUnknownOrMixed:
    return "UnknownOrMixed";
  case MissObstacleRootCause::kReasonerNotBlocked:
    return "ReasonerNotBlocked";
  }
  return "UnknownOrMixed";
}

} // namespace passable_area::tools
