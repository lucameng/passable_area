#ifndef PASSABLE_AREA_CORE_PIPELINE_PROCESSOR_HPP_
#define PASSABLE_AREA_CORE_PIPELINE_PROCESSOR_HPP_

#include "passable_area/core/features/terrain_feature_updater.hpp"
#include "passable_area/core/frontend/polar_frontend.hpp"
#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/observability/frame_observability_estimator.hpp"
#include "passable_area/core/preprocess/frame_preprocessor.hpp"
#include "passable_area/core/traversability/traversability_solver.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

namespace passable_area::core {

class Processor {
public:
  explicit Processor(const Config &config);

  FrameOutput update(const FrameInput &input);
  const Config &config() const { return config_; }

private:
  FrameOutput buildOutput(const FrameInput &frame,
                          const FrameObservability &observability) const;

  Config config_;
  FramePreprocessor preprocessor_;
  FrameObservabilityEstimator observability_estimator_;
  PolarFrontend frontend_;
  LocalTerrainMap map_;
  DropoutAwareMapUpdater map_updater_;
  TerrainFeatureUpdater feature_updater_;
  TraversabilitySolver traversability_solver_;
  Timestamp last_stamp_ = 0;
};

} // namespace passable_area::core

#endif
