#ifndef PASSABLE_AREA_TOOLS_FALSE_OBSTACLE_ANALYZER_HPP_
#define PASSABLE_AREA_TOOLS_FALSE_OBSTACLE_ANALYZER_HPP_

#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace passable_area::tools {

struct FalseObstacleDetectionBox {
  float x_min = 0.0f;
  float x_max = 0.0f;
  float y_min = 0.0f;
  float y_max = 0.0f;
};

enum class FalseObstacleRootCause : uint8_t {
  kClearanceDriven = 0,
  kObstacleEvidenceDriven = 1,
  kObstacleEvidencePlusLowContinuity = 2,
  kObservabilityInfluenced = 3,
  kUnknownOrMixed = 4,
};

struct FalseObstacleAnalyzerConfig {
  FalseObstacleDetectionBox detection_box;
  float hotspot_bin_size = 0.10f;
  float obstacle_evidence_high_threshold = 0.4f;
  float support_continuity_low_threshold = 0.3f;
  float mixed_severity_ratio = 0.1f;
};

struct FalseObstacleHotspot {
  float x = 0.0f;
  float y = 0.0f;
  float min_z = 0.0f;
  float max_z = 0.0f;
  int obstacle_point_count = 0;
  int source_cell = -1;
  int center_cell = -1;
  bool source_matches_center = false;
  float severity = 0.0f;
  float obstacle_evidence = 0.0f;
  float clearance = 0.0f;
  float support_continuity = 0.0f;
  float overhead_height = 0.0f;
  float support_anchor_used = 0.0f;
  float source_obstacle_evidence = 0.0f;
  float source_overhead_height = 0.0f;
  float source_support_anchor_used = 0.0f;
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
  bool facade_upper_edge_aligned_with_supported_neighbors = false;
  bool has_grid_values = false;
  bool has_observability = false;
  passable_area::core::ObservabilityState observability_state =
      passable_area::core::ObservabilityState::kObserved;
  FalseObstacleRootCause classification = FalseObstacleRootCause::kUnknownOrMixed;
  std::string explanation;
};

struct FalseObstacleFrameAnalysis {
  passable_area::core::Timestamp stamp = 0;
  double start_offset_sec = 0.0;
  int in_box_obstacle_point_count = 0;
  float severity = 0.0f;
  FalseObstacleRootCause classification = FalseObstacleRootCause::kUnknownOrMixed;
  bool frame_partial = false;
  bool rear_dropout = false;
  float max_local_obstacle_evidence = 0.0f;
  float min_local_clearance = 0.0f;
  float min_local_support_continuity = 0.0f;
  int rejected_suspicious_cell_count = 0;
  std::vector<FalseObstacleHotspot> hotspots;
};

struct FalseObstacleBagSummary {
  FalseObstacleDetectionBox detection_box;
  int total_frames = 0;
  int candidate_frames = 0;
  int rear_dropout_frames = 0;
  int longest_consecutive_candidate_run = 0;
  std::array<int, 5> root_cause_counts = {0, 0, 0, 0, 0};
  std::vector<FalseObstacleFrameAnalysis> ranked_frames;
};

class FalseObstacleAnalyzer {
public:
  FalseObstacleAnalyzer(const passable_area::core::Config &config,
                        const FalseObstacleAnalyzerConfig &analysis_config);

  std::optional<FalseObstacleFrameAnalysis> analyzeFrame(
      const passable_area::core::FrameOutput &output) const;

  FalseObstacleBagSummary buildSummary(
      int total_frames, int rear_dropout_frames, int longest_consecutive_candidate_run,
      std::vector<FalseObstacleFrameAnalysis> candidate_frames, int top_k) const;

  const FalseObstacleAnalyzerConfig &analysisConfig() const { return analysis_config_; }

private:
  struct LocalCellContext {
    bool has_grid_values = false;
    int source_cell = -1;
    int center_cell = -1;
    float obstacle_evidence = 0.0f;
    float clearance = 0.0f;
    float support_continuity = 0.0f;
    float overhead_height = 0.0f;
    float support_anchor_used = 0.0f;
    float source_obstacle_evidence = 0.0f;
    float source_overhead_height = 0.0f;
    float source_support_anchor_used = 0.0f;
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
    bool facade_upper_edge_aligned_with_supported_neighbors = false;
    bool has_observability = false;
    passable_area::core::ObservabilityState observability_state =
        passable_area::core::ObservabilityState::kObserved;
  };

  LocalCellContext lookupLocalContext(const passable_area::core::FrameOutput &output, float x,
                                      float y, int source_cell) const;
  FalseObstacleHotspot buildHotspot(const passable_area::core::FrameOutput &output, float x,
                                    float y, float min_z, float max_z, int source_cell,
                                    int obstacle_point_count) const;
  float computeHotspotSeverity(const FalseObstacleHotspot &hotspot) const;
  FalseObstacleRootCause classifyHotspot(const FalseObstacleHotspot &hotspot) const;
  FalseObstacleRootCause reduceFrameClass(const std::vector<FalseObstacleHotspot> &hotspots) const;

  passable_area::core::Config config_;
  FalseObstacleAnalyzerConfig analysis_config_;
};

const char *ToString(FalseObstacleRootCause cause);
const char *ToString(passable_area::core::ObservabilityState state);

} // namespace passable_area::tools

#endif
