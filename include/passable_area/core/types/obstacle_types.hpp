#ifndef PASSABLE_AREA_CORE_TYPES_OBSTACLE_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_OBSTACLE_TYPES_HPP_

#include <cstdint>

namespace passable_area::core {

enum class BlockReason : uint8_t {
  kNone = 0,
  kProtrusion,
  kLowClearance,
  kGeometryFailure,
  kMixed,
};

enum class ObstacleEvidenceStage : uint8_t {
  kNone = 0,
  kEvidenceLow,
  kBlocking,
};

enum class ObstaclePointPublishStatus : uint8_t {
  kNotApplicable = 0,
  kPublishedByProtrusion,
  kPublishedByOverhead,
  kGatedByEvidence,
  kGatedByHeight,
  kBlockedButNoSamples,
};

} // namespace passable_area::core

#endif
