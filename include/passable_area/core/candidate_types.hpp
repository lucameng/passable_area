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
  bool obstacle_overlap = false;
  int support_sample_count = 0;
  int obstacle_sample_count = 0;
};

struct ProtrusionCandidate {
  int cell = -1;
  float z = 0.0f;
  float evidence = 0.0f;
  float gain_scale = 1.0f;
};

struct OverheadCandidate {
  int cell = -1;
  float z = 0.0f;
  float evidence = 0.0f;
  float gain_scale = 1.0f;
};

struct FrontendOutput {
  std::vector<SupportCandidate> support_candidates;
  std::vector<ProtrusionCandidate> protrusion_candidates;
  std::vector<OverheadCandidate> overhead_candidates;
  std::vector<float> raw_sample_min_z;
  std::vector<float> raw_sample_max_z;
  std::vector<uint16_t> raw_sample_count;
  std::vector<float> filtered_sample_min_z;
  std::vector<float> filtered_sample_max_z;
  std::vector<uint16_t> filtered_sample_count;
  std::vector<uint8_t> obstacle_suspicious;
  std::vector<uint8_t> obstacle_candidate_cell;
};

} // namespace passable_area::core

#endif
