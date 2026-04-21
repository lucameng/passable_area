#include "passable_area/core/obstacle_reasoner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace passable_area::core {

ObstacleReasonerOutput
ObstacleReasoner::evaluate(const TerrainLayers &layers) const {
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
    const float clearance = cell < layers.clearance.size()
                                ? layers.clearance[cell]
                                : std::numeric_limits<float>::quiet_NaN();
    const float continuity = cell < layers.support_continuity.size()
                                 ? layers.support_continuity[cell]
                                 : 0.0f;
    const float slope = cell < layers.slope.size() ? layers.slope[cell] : 0.0f;
    const float step_up =
        cell < layers.step_up.size() ? layers.step_up[cell] : 0.0f;
    const float step_down =
        cell < layers.step_down.size() ? layers.step_down[cell] : 0.0f;
    const float roughness =
        cell < layers.roughness.size() ? layers.roughness[cell] : 0.0f;

    const bool protrusion_evidence_high =
        protrusion_evidence >= config_.obstacle_points_min_evidence;
    const bool overhead_evidence_high =
        overhead_evidence >= config_.obstacle_points_min_evidence;
    const bool low_clearance =
        std::isfinite(clearance) && clearance < config_.geometry.min_clearance;
    const float protrusion_height = cell < layers.protrusion_height.size()
                                        ? layers.protrusion_height[cell]
                                        : 0.0f;
    const float support_height = cell < layers.support_height.size()
                                    ? layers.support_height[cell]
                                    : 0.0f;
    const bool tall_protrusion =
        std::isfinite(protrusion_height) && std::isfinite(support_height) &&
        (protrusion_height - support_height) > config_.geometry.max_step_up;
    const bool protrusion_blocking =
        protrusion_evidence > 0.4f && (continuity < 0.3f || tall_protrusion);
    const bool geometry_failure =
        slope > config_.geometry.max_support_slope_deg ||
        step_up > config_.geometry.max_step_up ||
        step_down > config_.geometry.max_step_down ||
        roughness > config_.geometry.max_support_roughness;

    if (protrusion_evidence > 0.0f) {
      output.protrusion_stage[cell] = static_cast<uint8_t>(
          protrusion_blocking ? ObstacleEvidenceStage::kBlocking
                              : ObstacleEvidenceStage::kEvidenceLow);
    }
    if (overhead_evidence > 0.0f || low_clearance) {
      output.overhead_stage[cell] = static_cast<uint8_t>(
          low_clearance ? ObstacleEvidenceStage::kBlocking
                        : ObstacleEvidenceStage::kEvidenceLow);
    }

    BlockReason reason = BlockReason::kNone;
    if (low_clearance && protrusion_blocking) {
      reason = BlockReason::kMixed;
    } else if (low_clearance) {
      reason = BlockReason::kLowClearance;
    } else if (protrusion_blocking) {
      reason = BlockReason::kProtrusion;
    } else if (geometry_failure) {
      reason = BlockReason::kGeometryFailure;
    }
    output.block_reason[cell] = static_cast<uint8_t>(reason);

    ObstaclePointPublishStatus publish_status =
        ObstaclePointPublishStatus::kNotApplicable;
    if (reason == BlockReason::kProtrusion || reason == BlockReason::kMixed) {
      publish_status = protrusion_evidence_high
                           ? ObstaclePointPublishStatus::kPublishedByProtrusion
                           : ObstaclePointPublishStatus::kGatedByEvidence;
    } else if (reason == BlockReason::kLowClearance) {
      if (clearance > config_.geometry.max_step_up) {
        publish_status = overhead_evidence_high
                             ? ObstaclePointPublishStatus::kPublishedByOverhead
                             : ObstaclePointPublishStatus::kGatedByEvidence;
      } else {
        publish_status = ObstaclePointPublishStatus::kGatedByHeight;
      }
    }
    output.obstacle_point_publish_status[cell] =
        static_cast<uint8_t>(publish_status);
  }

  return output;
}

} // namespace passable_area::core
