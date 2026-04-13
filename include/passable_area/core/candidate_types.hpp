#ifndef PASSABLE_AREA_CORE_CANDIDATE_TYPES_HPP_
#define PASSABLE_AREA_CORE_CANDIDATE_TYPES_HPP_

#include "passable_area/core/types/basic_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <vector>

namespace passable_area::core {

struct SupportCandidate {
  int cell = -1;
  float z = 0.0f;
  float confidence = 0.0f;
};

struct ObstacleCandidate {
  int cell = -1;
  float z = 0.0f;
  float evidence = 0.0f;
  float gain_scale = 1.0f;
};

struct AmbiguousCandidate {
  int cell = -1;
  float z = 0.0f;
};

struct FrontendOutput {
  std::vector<SupportCandidate> support_candidates;
  std::vector<ObstacleCandidate> obstacle_candidates;
  std::vector<AmbiguousCandidate> ambiguous_candidates;
  std::vector<float> support_anchor_used;
  std::vector<uint8_t> support_anchor_origin;
  std::vector<uint8_t> support_anchor_authority;
  std::vector<uint8_t> anchor_leak_suppression_enabled;
  std::vector<uint16_t> sub_support_leak_count;
  std::vector<uint16_t> anchor_below_observation_count;
  std::vector<uint16_t> stale_anchor_residual_filtered_count;
  std::vector<float> raw_sample_min_z;
  std::vector<float> raw_sample_max_z;
  std::vector<uint16_t> raw_sample_count;
  std::vector<float> filtered_sample_min_z;
  std::vector<float> filtered_sample_max_z;
  std::vector<uint16_t> filtered_sample_count;
  // Raw local upper-band fact relative to the current cell support_ref.
  std::vector<uint8_t> raw_upper_support_cell;
  // Explanation-adjusted upper-support mask used by neighborhood confirmation.
  std::vector<uint8_t> explanation_adjusted_upper_support_cell;
  // Compatibility alias kept equal to explanation_adjusted_upper_support_cell.
  std::vector<uint8_t> upper_support_cell;
  std::vector<uint8_t> obstacle_local_triggered;
  std::vector<uint8_t> obstacle_upper_patch_confirmed;
  std::vector<uint8_t> obstacle_explanation_rejected;
  std::vector<uint8_t> obstacle_suspicious;
  std::vector<uint8_t> obstacle_candidate_cell;
  // Legacy compatibility flag. It is set when a locally triggered cell fails
  // upper-patch confirmation or is explicitly rejected during explanation.
  // Use obstacle_upper_patch_confirmed / obstacle_explanation_rejected /
  // explanation_decision as the primary stage indicators.
  std::vector<uint8_t> obstacle_rejected_by_neighbor_support;
  std::vector<int8_t> neighbor_upper_support_count;
  std::vector<int8_t> aligned_neighbor_support_count;
  std::vector<uint8_t> explanation_decision;
  std::vector<uint8_t> facade_lower_upper_coexisting;
  std::vector<uint8_t> facade_upper_edge_aligned_with_supported_neighbors;
};

} // namespace passable_area::core

#endif
