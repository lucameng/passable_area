#ifndef PASSABLE_AREA_CORE_FRAME_OBSERVABILITY_ESTIMATOR_HPP_
#define PASSABLE_AREA_CORE_FRAME_OBSERVABILITY_ESTIMATOR_HPP_

#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

namespace passable_area::core {

class FrameObservabilityEstimator {
public:
  explicit FrameObservabilityEstimator(const Config &config) : config_(config) {}

  FrameObservability estimate(const ProcessedFrame &frame) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
