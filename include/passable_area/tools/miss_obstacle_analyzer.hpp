#ifndef PASSABLE_AREA_TOOLS_MISS_OBSTACLE_ANALYZER_HPP_
#define PASSABLE_AREA_TOOLS_MISS_OBSTACLE_ANALYZER_HPP_

#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace passable_area::tools {

struct MissObstacleDetectionBox {
  float x_min = 0.0f;
  float x_max = 0.0f;
  float y_min = 0.0f;
  float y_max = 0.0f;
};

enum class MissObstacleRootCause : uint8_t {
  kNoSamplesInRoi = 0,
  kNoFrontendCandidate = 1,
  kNoFrontendObstacleSuspicion = kNoFrontendCandidate,
  kEvidenceTooLow = 2,
  kObstacleEvidenceTooLow = kEvidenceTooLow,
  kPublishHeightGated = 3,
  kOutputHeightGateNotMet = kPublishHeightGated,
  kNoObstacleSourceSamplesInRoi = 4,
  kUnknownOrMixed = 5,
  kReasonerNotBlocked = 6,
};

struct MissObstacleAnalyzerConfig {
  MissObstacleDetectionBox detection_box;
  int representative_cell_limit = 3;
};

struct MissObstacleRepresentativeCell {
  float x = 0.0f;
  float y = 0.0f;
  float min_sample_relative_z = 0.0f;
  float max_sample_relative_z = 0.0f;
  int sample_count = 0;
  float support_height = 0.0f;
  float support_ref = 0.0f;
  float overhead_height = 0.0f;
  float protrusion_evidence = 0.0f;
  float overhead_evidence = 0.0f;
  float obstacle_evidence = 0.0f;
  uint8_t block_reason = 0U;
  uint8_t obstacle_point_publish_status = 0U;
  float support_confidence = 0.0f;
  float support_continuity = 0.0f;
  float max_sample_z_minus_support_ref = 0.0f;
  float raw_sample_min_z = 0.0f;
  float raw_sample_max_z = 0.0f;
  uint16_t raw_sample_count = 0U;
  float filtered_sample_min_z = 0.0f;
  float filtered_sample_max_z = 0.0f;
  uint16_t filtered_sample_count = 0U;
  bool obstacle_suspicious = false;
  bool obstacle_candidate_cell = false;
  std::string explanation;
};

struct MissObstacleFrameAnalysis {
  passable_area::core::Timestamp stamp = 0;
  double start_offset_sec = 0.0;
  int roi_sample_count = 0;
  int roi_cells_with_any_samples = 0;
  int roi_obstacle_point_count = 0;
  int obstacle_suspicious_cell_count = 0;
  int obstacle_candidate_cell_count = 0;
  float max_obstacle_evidence = 0.0f;
  float max_support_confidence = 0.0f;
  float min_clearance = 0.0f;
  float min_support_continuity = 0.0f;
  float severity = 0.0f;
  MissObstacleRootCause classification = MissObstacleRootCause::kUnknownOrMixed;
  std::string explanation;
  std::vector<std::string> evidence_lines;
  std::vector<MissObstacleRepresentativeCell> representative_cells;
};

struct MissObstacleBagSummary {
  MissObstacleDetectionBox detection_box;
  int total_frames = 0;
  int missed_frames = 0;
  std::array<int, 7> root_cause_counts = {0, 0, 0, 0, 0, 0, 0};
  std::vector<MissObstacleFrameAnalysis> ranked_frames;
};

class MissObstacleAnalyzer {
public:
  MissObstacleAnalyzer(const passable_area::core::Config &config,
                       const MissObstacleAnalyzerConfig &analysis_config);

  std::optional<MissObstacleFrameAnalysis> analyzeFrame(
      const passable_area::core::FrameOutput &output,
      const passable_area::core::ProcessedFrame &processed_frame) const;

  MissObstacleBagSummary
  buildSummary(int total_frames,
               std::vector<MissObstacleFrameAnalysis> candidate_frames) const;

  const MissObstacleAnalyzerConfig &analysisConfig() const {
    return analysis_config_;
  }

private:
  passable_area::core::Config config_;
  MissObstacleAnalyzerConfig analysis_config_;
};

const char *ToString(MissObstacleRootCause cause);

} // namespace passable_area::tools

#endif
