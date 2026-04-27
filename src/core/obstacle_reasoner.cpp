#include "passable_area/core/obstacle_reasoner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace passable_area::core {
namespace {

constexpr int kDenseCurrentFrameSampleCountSlack = 1;

} // namespace

bool ObstacleReasoner::hasDenseCurrentFrameProtrusionSource(
    float protrusion_evidence, size_t cell,
    const ObstaclePublicationContext &publication_context) const {
  if (publication_context.obstacle_candidate_cell == nullptr ||
      cell >= publication_context.obstacle_candidate_cell->size() ||
      (*publication_context.obstacle_candidate_cell)[cell] == 0U) {
    return false;
  }
  if (publication_context.raw_sample_count == nullptr ||
      cell >= publication_context.raw_sample_count->size()) {
    return false;
  }
  const int dense_source_count =
      std::max(1, config_.observability.min_points_per_sector -
                       kDenseCurrentFrameSampleCountSlack);
  if ((*publication_context.raw_sample_count)[cell] < dense_source_count) {
    return false;
  }
  if (publication_context.current_protrusion_evidence_gain == nullptr ||
      cell >= publication_context.current_protrusion_evidence_gain->size()) {
    return false;
  }
  const float current_frame_gain =
      (*publication_context.current_protrusion_evidence_gain)[cell];
  return current_frame_gain > 0.0f && protrusion_evidence >= current_frame_gain;
}

bool ObstacleReasoner::hasActiveOverheadState(float overhead_confidence,
                                              float overhead_evidence) const {
  const float state_signal = std::max(overhead_confidence, overhead_evidence);
  return state_signal > 0.0f &&
         state_signal >= config_.persistence.obstacle_height_clear_threshold;
}

ObstacleReasonerOutput
ObstacleReasoner::evaluate(
    const TerrainLayers &layers,
    const ObstaclePublicationContext &publication_context) const {
  const size_t cell_count = layers.support_height.size();
  ObstacleReasonerOutput output;
  output.block_reason.assign(cell_count,
                             static_cast<uint8_t>(BlockReason::kNone));
  output.protrusion_stage.assign(
      cell_count, static_cast<uint8_t>(ObstacleEvidenceStage::kNone));
  output.overhead_stage.assign(
      cell_count, static_cast<uint8_t>(ObstacleEvidenceStage::kNone));
  output.obstacle_point_publish_status.assign(
      cell_count,
      static_cast<uint8_t>(ObstaclePointPublishStatus::kNotApplicable));

  for (size_t cell = 0; cell < cell_count; ++cell) {
    const float protrusion_evidence = cell < layers.protrusion_evidence.size()
                                          ? layers.protrusion_evidence[cell]
                                          : 0.0f;
    const float overhead_evidence = cell < layers.overhead_evidence.size()
                                        ? layers.overhead_evidence[cell]
                                        : 0.0f;
    const float overhead_confidence = cell < layers.overhead_confidence.size()
                                          ? layers.overhead_confidence[cell]
                                          : 0.0f;
    const float clearance = cell < layers.clearance.size()
                                ? layers.clearance[cell]
                                : std::numeric_limits<float>::quiet_NaN();
    const float slope = cell < layers.slope.size() ? layers.slope[cell] : 0.0f;
    const float step_up =
        cell < layers.step_up.size() ? layers.step_up[cell] : 0.0f;
    const float step_down =
        cell < layers.step_down.size() ? layers.step_down[cell] : 0.0f;
    const float roughness =
        cell < layers.roughness.size() ? layers.roughness[cell] : 0.0f;

    const bool protrusion_evidence_high =
        protrusion_evidence >= config_.obstacle_points_min_evidence;
    const bool dense_protrusion_source =
        hasDenseCurrentFrameProtrusionSource(protrusion_evidence, cell,
                                             publication_context);
    const bool overhead_evidence_high =
        overhead_evidence >= config_.obstacle_points_min_evidence;
    const bool low_clearance_geometry =
        std::isfinite(clearance) && clearance < config_.geometry.min_clearance;
    const bool overhead_state_active =
        hasActiveOverheadState(overhead_confidence, overhead_evidence);
    const bool low_clearance =
        low_clearance_geometry && overhead_state_active;
    const float protrusion_height = cell < layers.protrusion_height.size()
                                        ? layers.protrusion_height[cell]
                                        : 0.0f;
    const float support_height = cell < layers.support_height.size()
                                    ? layers.support_height[cell]
                                    : 0.0f;
    const float current_obstacle_candidate_height =
        publication_context.current_obstacle_candidate_height != nullptr &&
                cell <
                    publication_context.current_obstacle_candidate_height->size()
            ? (*publication_context.current_obstacle_candidate_height)[cell]
            : std::numeric_limits<float>::quiet_NaN();
    float publish_height_ref = protrusion_height;
    if (std::isfinite(current_obstacle_candidate_height)) {
      publish_height_ref = std::isfinite(publish_height_ref)
                               ? std::max(publish_height_ref,
                                          current_obstacle_candidate_height)
                               : current_obstacle_candidate_height;
    }
    const bool tall_protrusion =
        std::isfinite(publish_height_ref) && std::isfinite(support_height) &&
        (publish_height_ref - support_height) > config_.geometry.max_step_up;
    const bool protrusion_blocking =
        (protrusion_evidence_high || dense_protrusion_source) &&
        tall_protrusion;
    const bool geometry_failure =
        slope > config_.geometry.max_support_slope_deg ||
        step_up > config_.geometry.max_step_up ||
        step_down > config_.geometry.max_step_down ||
        roughness > config_.geometry.max_support_roughness;
    const bool geometry_publishable_protrusion =
        geometry_failure && tall_protrusion &&
        (protrusion_evidence_high || dense_protrusion_source);

    if (protrusion_evidence > 0.0f) {
      output.protrusion_stage[cell] = static_cast<uint8_t>(
          protrusion_blocking ? ObstacleEvidenceStage::kBlocking
                              : ObstacleEvidenceStage::kEvidenceLow);
    }
    if (overhead_evidence > 0.0f || overhead_confidence > 0.0f ||
        low_clearance_geometry) {
      output.overhead_stage[cell] = static_cast<uint8_t>(
          low_clearance ? ObstacleEvidenceStage::kBlocking
                        : ObstacleEvidenceStage::kEvidenceLow);
    }

    BlockReason reason = BlockReason::kNone;
    if (low_clearance && protrusion_blocking) {
      reason = BlockReason::kMixed;
    } else if (low_clearance) {
      reason = BlockReason::kLowClearance;
    } else if (geometry_publishable_protrusion && !protrusion_evidence_high) {
      reason = BlockReason::kGeometryFailure;
    } else if (protrusion_blocking) {
      reason = BlockReason::kProtrusion;
    } else if (geometry_failure) {
      reason = BlockReason::kGeometryFailure;
    }
    output.block_reason[cell] = static_cast<uint8_t>(reason);

    ObstaclePointPublishStatus publish_status =
        ObstaclePointPublishStatus::kNotApplicable;
    if (reason == BlockReason::kProtrusion || reason == BlockReason::kMixed) {
      if (protrusion_evidence_high) {
        publish_status = ObstaclePointPublishStatus::kPublishedByProtrusion;
      } else if (dense_protrusion_source) {
        publish_status =
            ObstaclePointPublishStatus::kPublishedByDenseProtrusion;
      } else {
        publish_status = ObstaclePointPublishStatus::kGatedByEvidence;
      }
    } else if (reason == BlockReason::kLowClearance) {
      if (clearance > config_.geometry.max_step_up) {
        publish_status = overhead_evidence_high
                             ? ObstaclePointPublishStatus::kPublishedByOverhead
                             : ObstaclePointPublishStatus::kGatedByEvidence;
      } else {
        publish_status = ObstaclePointPublishStatus::kGatedByHeight;
      }
    } else if (reason == BlockReason::kGeometryFailure &&
               geometry_publishable_protrusion) {
      publish_status =
          ObstaclePointPublishStatus::kPublishedByGeometryFailure;
    }
    output.obstacle_point_publish_status[cell] =
        static_cast<uint8_t>(publish_status);
  }

  return output;
}

} // namespace passable_area::core
