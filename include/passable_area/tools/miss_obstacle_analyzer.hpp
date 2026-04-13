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
  kNoFrontendObstacleSuspicion = 1,
  kRejectedByNeighborSupport = 2,
  kLeakFilteredToNoCandidate = 3,
  kObstacleEvidenceTooLow = 4,
  kOutputHeightGateNotMet = 5,
  kNoObstacleSourceSamplesInRoi = 6,
  kUnknownOrMixed = 7,
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
  float obstacle_evidence = 0.0f;
  float support_confidence = 0.0f;
  float support_anchor_used = 0.0f;
  float max_sample_z_minus_support_ref = 0.0f;
  uint16_t sub_support_leak_count = 0U;
  bool raw_upper_support_cell = false;
  bool adjusted_upper_support_cell = false;
  bool upper_support_cell = false;
  bool obstacle_local_triggered = false;
  bool obstacle_upper_patch_confirmed = false;
  bool obstacle_explanation_rejected = false;
  bool obstacle_suspicious = false;
  bool obstacle_candidate_cell = false;
  bool obstacle_rejected_by_neighbor_support = false;
  int8_t neighbor_upper_support_count = 0;
  int8_t aligned_neighbor_support_count = 0;
  uint8_t explanation_decision = 0U;
  bool facade_lower_upper_coexisting = false;
  bool facade_stable_upper_edge_without_support_lift = false;
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
  int rejected_suspicious_cell_count = 0;
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
  std::array<int, 8> root_cause_counts = {0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<MissObstacleFrameAnalysis> ranked_frames;
};

class MissObstacleAnalyzer {
public:
  MissObstacleAnalyzer(const passable_area::core::Config &config,
                       const MissObstacleAnalyzerConfig &analysis_config);

  std::optional<MissObstacleFrameAnalysis> analyzeFrame(
      const passable_area::core::FrameOutput &output,
      const passable_area::core::ProcessedFrame &processed_frame) const;

  MissObstacleBagSummary buildSummary(int total_frames,
                                      std::vector<MissObstacleFrameAnalysis> candidate_frames) const;

  const MissObstacleAnalyzerConfig &analysisConfig() const { return analysis_config_; }

private:
  passable_area::core::Config config_;
  MissObstacleAnalyzerConfig analysis_config_;
};

const char *ToString(MissObstacleRootCause cause);

} // namespace passable_area::tools

#endif
