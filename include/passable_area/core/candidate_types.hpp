#ifndef PASSABLE_AREA_CORE_CANDIDATE_TYPES_HPP_
#define PASSABLE_AREA_CORE_CANDIDATE_TYPES_HPP_

#include "passable_area/core/types/basic_types.hpp"

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
  std::vector<uint16_t> sub_support_leak_count;
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
  std::vector<uint8_t> obstacle_rejected_by_neighbor_support;
  std::vector<int8_t> neighbor_upper_support_count;
  std::vector<int8_t> aligned_neighbor_support_count;
  std::vector<uint8_t> explanation_decision;
  std::vector<uint8_t> facade_lower_upper_coexisting;
  std::vector<uint8_t> facade_stable_upper_edge_without_support_lift;
};

} // namespace passable_area::core

#endif
