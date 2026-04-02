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
  int obstacle_point_count = 0;
  float severity = 0.0f;
  float obstacle_evidence = 0.0f;
  float clearance = 0.0f;
  float support_continuity = 0.0f;
  float overhead_height = 0.0f;
  bool has_grid_values = false;
  bool has_observability = false;
  passable_area::core::ObservabilityState observability_state =
      passable_area::core::ObservabilityState::kObserved;
  FalseObstacleRootCause classification = FalseObstacleRootCause::kUnknownOrMixed;
  std::string explanation;
};

struct FalseObstacleFrameAnalysis {
  passable_area::core::Timestamp stamp = 0;
  int in_box_obstacle_point_count = 0;
  float severity = 0.0f;
  FalseObstacleRootCause classification = FalseObstacleRootCause::kUnknownOrMixed;
  bool frame_partial = false;
  bool rear_dropout = false;
  float max_local_obstacle_evidence = 0.0f;
  float min_local_clearance = 0.0f;
  float min_local_support_continuity = 0.0f;
  std::vector<FalseObstacleHotspot> hotspots;
};

struct FalseObstacleBagSummary {
  FalseObstacleDetectionBox detection_box;
  int total_frames = 0;
  int candidate_frames = 0;
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
      int total_frames, int longest_consecutive_candidate_run,
      std::vector<FalseObstacleFrameAnalysis> candidate_frames, int top_k) const;

  const FalseObstacleAnalyzerConfig &analysisConfig() const { return analysis_config_; }

private:
  struct LocalCellContext {
    bool has_grid_values = false;
    float obstacle_evidence = 0.0f;
    float clearance = 0.0f;
    float support_continuity = 0.0f;
    float overhead_height = 0.0f;
    bool has_observability = false;
    passable_area::core::ObservabilityState observability_state =
        passable_area::core::ObservabilityState::kObserved;
  };

  LocalCellContext lookupLocalContext(const passable_area::core::FrameOutput &output, float x,
                                      float y) const;
  FalseObstacleHotspot buildHotspot(const passable_area::core::FrameOutput &output, float x,
                                    float y, int obstacle_point_count) const;
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
