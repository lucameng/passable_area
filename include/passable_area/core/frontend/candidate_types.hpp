#ifndef PASSABLE_AREA_CORE_FRONTEND_CANDIDATE_TYPES_HPP_
#define PASSABLE_AREA_CORE_FRONTEND_CANDIDATE_TYPES_HPP_

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
};

struct AmbiguousCandidate {
  int cell = -1;
  float z = 0.0f;
};

struct FrontendOutput {
  std::vector<SupportCandidate> support_candidates;
  std::vector<ObstacleCandidate> obstacle_candidates;
  std::vector<AmbiguousCandidate> ambiguous_candidates;
};

} // namespace passable_area::core

#endif
