#ifndef PASSABLE_AREA_CORE_PROCESSOR_HPP_
#define PASSABLE_AREA_CORE_PROCESSOR_HPP_

#include "passable_area/core/frame_observability_estimator.hpp"
#include "passable_area/core/frame_preprocessor.hpp"
#include "passable_area/core/mapping/dropout_aware_map_updater.hpp"
#include "passable_area/core/mapping/local_terrain_map.hpp"
#include "passable_area/core/obstacle_reasoner.hpp"
#include "passable_area/core/polar_frontend.hpp"
#include "passable_area/core/terrain_feature_updater.hpp"
#include "passable_area/core/traversability_solver.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

namespace passable_area::core {

class Processor {
public:
  explicit Processor(const Config &config);

  FrameOutput update(const FrameInput &input);
  const Config &config() const { return config_; }

private:
  struct CachedRearObstaclePoint {
    Point3f point_in_map;
    int source_cell = -1;
  };

  struct RearObstaclePointCache {
    bool valid = false;
    bool consumed_for_bridge = false;
    Timestamp stamp = 0;
    Pose3D base_pose_in_map;
    std::vector<CachedRearObstaclePoint> points;
  };

  FrameOutput buildOutput(const ProcessedFrame &frame,
                          const FrameObservability &observability,
                          const FrontendOutput &frontend_output,
                          const ObstacleReasonerOutput &reasoner_output);

  Config config_;
  FramePreprocessor preprocessor_;
  FrameObservabilityEstimator observability_estimator_;
  PolarFrontend frontend_;
  LocalTerrainMap map_;
  DropoutAwareMapUpdater map_updater_;
  TerrainFeatureUpdater feature_updater_;
  TraversabilitySolver traversability_solver_;
  ObstacleReasoner obstacle_reasoner_;
  Timestamp last_stamp_ = 0;
  RearObstaclePointCache rear_obstacle_point_cache_;
};

} // namespace passable_area::core

#endif
