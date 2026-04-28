#ifndef PASSABLE_AREA_CORE_OBSTACLE_PUBLICATION_HPP_
#define PASSABLE_AREA_CORE_OBSTACLE_PUBLICATION_HPP_

#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/obstacle_types.hpp"

#include <cstdint>

namespace passable_area::core {

struct ObstaclePublicationSample {
  float z_in_map = 0.0f;
  float z_in_base_link = 0.0f;
  float z_in_base_gravity = 0.0f;
};

struct ObstaclePublicationHeightGateTrace {
  bool finite_support_ref = false;
  bool above_min_height = false;
  bool above_step_height = false;
  bool below_base_link_ceiling = false;
  bool above_publish_path_floor = true;
  bool passes = false;
};

struct ObstaclePublicationSampleStats {
  bool has_samples = false;
  int sample_count = 0;
  int publishable_sample_count = 0;
  bool any_above_min_height = false;
  bool any_above_step_height = false;
  bool any_below_base_link_ceiling = false;
  bool any_above_publish_path_floor = false;
  float max_z_in_map = 0.0f;
  float min_z_in_base_link = 0.0f;
  float min_z_in_base_gravity = 0.0f;
};

struct ObstaclePublicationDecisionTrace {
  ObstaclePointPublishStatus reasoner_status =
      ObstaclePointPublishStatus::kNotApplicable;
  bool reasoner_requests_publication = false;
  bool has_current_samples = false;
  ObstaclePublicationHeightGateTrace height_gate;
  ObstaclePointPublishStatus final_status =
      ObstaclePointPublishStatus::kNotApplicable;
};

bool IsObstaclePointPublishStatusPublished(uint8_t status);
bool IsObstaclePointPublishStatusPublished(ObstaclePointPublishStatus status);
bool IsObstaclePointPublishStatusApplicable(uint8_t status);
bool IsGeometryFailurePublishStatus(uint8_t status);
bool IsLowClearanceBridgePublishStatus(uint8_t status);

const char *ObstaclePublicationPathName(uint8_t status);
const char *ObstaclePublicationPathName(ObstaclePointPublishStatus status);

ObstaclePublicationHeightGateTrace EvaluateObstaclePublicationSampleHeightGates(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSample &sample);

void AccumulateObstaclePublicationSample(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSample &sample,
    ObstaclePublicationSampleStats &sample_stats);

ObstaclePublicationHeightGateTrace EvaluateObstaclePublicationHeightGates(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSampleStats &sample_stats);

ObstaclePublicationDecisionTrace EvaluateObstaclePublicationDecisionTrace(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSampleStats &sample_stats);

ObstaclePointPublishStatus FinalizeObstaclePublicationStatus(
    uint8_t reasoner_status, bool saw_current_samples, bool published);

} // namespace passable_area::core

#endif
