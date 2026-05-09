#include "passable_area/core/obstacle_publication.hpp"

#include <algorithm>
#include <cmath>

namespace passable_area::core {

bool IsObstaclePointPublishStatusPublished(ObstaclePointPublishStatus status) {
  switch (status) {
  case ObstaclePointPublishStatus::kPublishedByProtrusion:
  case ObstaclePointPublishStatus::kPublishedByDenseProtrusion:
  case ObstaclePointPublishStatus::kPublishedByGeometryFailure:
  case ObstaclePointPublishStatus::kPublishedByOverhead:
    return true;
  case ObstaclePointPublishStatus::kNotApplicable:
  case ObstaclePointPublishStatus::kGatedByEvidence:
  case ObstaclePointPublishStatus::kGatedByHeight:
  case ObstaclePointPublishStatus::kBlockedButNoSamples:
    return false;
  }
  return false;
}

bool IsObstaclePointPublishStatusPublished(uint8_t status) {
  return IsObstaclePointPublishStatusPublished(
      static_cast<ObstaclePointPublishStatus>(status));
}

bool IsObstaclePointPublishStatusApplicable(uint8_t status) {
  return static_cast<ObstaclePointPublishStatus>(status) !=
         ObstaclePointPublishStatus::kNotApplicable;
}

bool IsGeometryFailurePublishStatus(uint8_t status) {
  return static_cast<ObstaclePointPublishStatus>(status) ==
         ObstaclePointPublishStatus::kPublishedByGeometryFailure;
}

bool IsLowClearanceBridgePublishStatus(uint8_t status) {
  return static_cast<ObstaclePointPublishStatus>(status) ==
         ObstaclePointPublishStatus::kPublishedByOverhead;
}

const char *ObstaclePublicationPathName(ObstaclePointPublishStatus status) {
  switch (status) {
  case ObstaclePointPublishStatus::kPublishedByProtrusion:
    return "ProtrusionEvidence";
  case ObstaclePointPublishStatus::kPublishedByDenseProtrusion:
    return "DenseProtrusionSource";
  case ObstaclePointPublishStatus::kPublishedByGeometryFailure:
    return "GeometryFailure";
  case ObstaclePointPublishStatus::kPublishedByOverhead:
    return "LowClearanceBridge";
  case ObstaclePointPublishStatus::kGatedByEvidence:
    return "GatedByEvidence";
  case ObstaclePointPublishStatus::kGatedByHeight:
    return "GatedByHeight";
  case ObstaclePointPublishStatus::kBlockedButNoSamples:
    return "BlockedButNoSamples";
  case ObstaclePointPublishStatus::kNotApplicable:
    return "None";
  }
  return "None";
}

const char *ObstaclePublicationPathName(uint8_t status) {
  return ObstaclePublicationPathName(
      static_cast<ObstaclePointPublishStatus>(status));
}

ObstaclePublicationHeightGateTrace EvaluateObstaclePublicationSampleHeightGates(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSample &sample) {
  ObstaclePublicationHeightGateTrace trace;
  trace.finite_support_ref = std::isfinite(support_ref);
  trace.above_min_height =
      trace.finite_support_ref &&
      sample.z_in_map >= support_ref + config.obstacle_points_min_height;
  trace.above_step_height =
      trace.finite_support_ref &&
      sample.z_in_map > support_ref + config.geometry.max_step_up;
  trace.below_base_link_ceiling =
      sample.z_in_base_link <= config.obstacle_points_max_height_in_base_link;
  trace.above_publish_path_floor =
      !IsGeometryFailurePublishStatus(reasoner_status) ||
      sample.z_in_base_gravity >= config.obstacle_points_min_height;
  trace.passes = trace.finite_support_ref && trace.above_min_height &&
                 trace.below_base_link_ceiling &&
                 trace.above_publish_path_floor;
  return trace;
}

void AccumulateObstaclePublicationSample(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSample &sample,
    ObstaclePublicationSampleStats &sample_stats) {
  if (!sample_stats.has_samples) {
    sample_stats.max_z_in_map = sample.z_in_map;
    sample_stats.min_z_in_base_link = sample.z_in_base_link;
    sample_stats.min_z_in_base_gravity = sample.z_in_base_gravity;
  } else {
    sample_stats.max_z_in_map =
        std::max(sample_stats.max_z_in_map, sample.z_in_map);
    sample_stats.min_z_in_base_link =
        std::min(sample_stats.min_z_in_base_link, sample.z_in_base_link);
    sample_stats.min_z_in_base_gravity =
        std::min(sample_stats.min_z_in_base_gravity, sample.z_in_base_gravity);
  }
  sample_stats.has_samples = true;
  ++sample_stats.sample_count;
  const auto sample_trace = EvaluateObstaclePublicationSampleHeightGates(
      config, reasoner_status, support_ref, sample);
  sample_stats.any_above_min_height =
      sample_stats.any_above_min_height || sample_trace.above_min_height;
  sample_stats.any_above_step_height =
      sample_stats.any_above_step_height || sample_trace.above_step_height;
  sample_stats.any_below_base_link_ceiling =
      sample_stats.any_below_base_link_ceiling ||
      sample_trace.below_base_link_ceiling;
  sample_stats.any_above_publish_path_floor =
      sample_stats.any_above_publish_path_floor ||
      sample_trace.above_publish_path_floor;
  if (sample_trace.passes) {
    ++sample_stats.publishable_sample_count;
  }
}

ObstaclePublicationHeightGateTrace EvaluateObstaclePublicationHeightGates(
    const Config &, uint8_t, float support_ref,
    const ObstaclePublicationSampleStats &sample_stats) {
  ObstaclePublicationHeightGateTrace trace;
  trace.finite_support_ref = std::isfinite(support_ref);
  if (!sample_stats.has_samples) {
    return trace;
  }
  trace.above_min_height = sample_stats.any_above_min_height;
  trace.above_step_height = sample_stats.any_above_step_height;
  trace.below_base_link_ceiling = sample_stats.any_below_base_link_ceiling;
  trace.above_publish_path_floor = sample_stats.any_above_publish_path_floor;
  trace.passes = sample_stats.publishable_sample_count > 0;
  return trace;
}

ObstaclePublicationDecisionTrace EvaluateObstaclePublicationDecisionTrace(
    const Config &config, uint8_t reasoner_status, float support_ref,
    const ObstaclePublicationSampleStats &sample_stats) {
  ObstaclePublicationDecisionTrace trace;
  trace.reasoner_status =
      static_cast<ObstaclePointPublishStatus>(reasoner_status);
  trace.reasoner_requests_publication =
      IsObstaclePointPublishStatusPublished(trace.reasoner_status) ||
      trace.reasoner_status == ObstaclePointPublishStatus::kGatedByHeight ||
      trace.reasoner_status == ObstaclePointPublishStatus::kBlockedButNoSamples;
  trace.has_current_samples = sample_stats.has_samples;
  trace.height_gate = EvaluateObstaclePublicationHeightGates(
      config, reasoner_status, support_ref, sample_stats);

  if (!IsObstaclePointPublishStatusPublished(trace.reasoner_status)) {
    trace.final_status = trace.reasoner_status;
  } else if (!trace.has_current_samples) {
    trace.final_status = ObstaclePointPublishStatus::kBlockedButNoSamples;
  } else if (!trace.height_gate.passes) {
    trace.final_status = ObstaclePointPublishStatus::kGatedByHeight;
  } else {
    trace.final_status = trace.reasoner_status;
  }
  return trace;
}

ObstaclePointPublishStatus
FinalizeObstaclePublicationStatus(uint8_t reasoner_status,
                                  bool saw_current_samples, bool published) {
  const auto status = static_cast<ObstaclePointPublishStatus>(reasoner_status);
  if (!IsObstaclePointPublishStatusPublished(status) || published) {
    return status;
  }
  return saw_current_samples ? ObstaclePointPublishStatus::kGatedByHeight
                             : ObstaclePointPublishStatus::kBlockedButNoSamples;
}

} // namespace passable_area::core
