#include "passable_area/core/config_validation.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace passable_area::core {
namespace {

bool IsPositiveFinite(float value) { return std::isfinite(value) && value > 0.0f; }

bool IsUnitInterval(float value) {
  return std::isfinite(value) && value >= 0.0f && value <= 1.0f;
}

void Require(bool condition, const std::string &message,
             std::vector<std::string> &errors) {
  if (!condition) {
    errors.push_back(message);
  }
}

} // namespace

std::vector<std::string> ValidateConfig(const Config &config) {
  std::vector<std::string> errors;

  Require(IsPositiveFinite(config.map.length), "map_length must be finite and > 0", errors);
  Require(IsPositiveFinite(config.map.width), "map_width must be finite and > 0", errors);
  Require(IsPositiveFinite(config.map.resolution), "map_resolution must be finite and > 0",
          errors);
  Require(std::isfinite(config.map.height_min) && std::isfinite(config.map.height_max) &&
              config.map.height_min < config.map.height_max,
          "map height window must be finite and height_min < height_max", errors);

  Require(IsPositiveFinite(config.geometry.max_support_slope_deg),
          "max_support_slope must be finite and > 0", errors);
  Require(IsPositiveFinite(config.geometry.max_step_up), "max_step_up must be finite and > 0",
          errors);
  Require(IsPositiveFinite(config.geometry.max_step_down),
          "max_step_down must be finite and > 0", errors);
  Require(IsPositiveFinite(config.geometry.max_support_roughness),
          "max_support_roughness must be finite and > 0", errors);
  Require(IsPositiveFinite(config.geometry.min_clearance),
          "min_clearance must be finite and > 0", errors);
  Require(config.geometry.min_clearance > config.geometry.max_step_up,
          "min_clearance must be greater than max_step_up", errors);
  Require(IsPositiveFinite(config.geometry.profile_split_gap),
          "profile_split_gap must be finite and > 0", errors);

  Require(config.observability.sector_count > 0, "sector_count must be > 0", errors);
  Require(config.observability.min_points_per_sector > 0,
          "min_points_per_sector must be > 0", errors);
  Require(config.observability.dropout_sector_gap_threshold >= 0,
          "dropout_sector_gap_threshold must be >= 0", errors);
  Require(IsUnitInterval(config.observability.partial_sector_ratio),
          "partial_sector_ratio must be in [0, 1]", errors);
  Require(IsPositiveFinite(config.observability.stale_to_unknown_time),
          "stale_to_unknown_time must be finite and > 0", errors);
  Require(IsUnitInterval(config.observability.min_support_confidence),
          "min_support_confidence must be in [0, 1]", errors);

  Require(config.persistence.support_persistence_frames > 0,
          "support_persistence_frames must be > 0", errors);
  Require(IsUnitInterval(config.persistence.support_confidence_gain),
          "support_confidence_gain must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.support_confidence_decay),
          "support_confidence_decay must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.obstacle_evidence_gain),
          "obstacle_evidence_gain must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.obstacle_evidence_decay),
          "obstacle_evidence_decay must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.obstacle_clear_observed_decay),
          "obstacle_clear_observed_decay must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.obstacle_clear_partial_decay_scale),
          "obstacle_clear_partial_decay_scale must be in [0, 1]", errors);
  Require(IsUnitInterval(config.persistence.obstacle_height_clear_threshold),
          "obstacle_height_clear_threshold must be in [0, 1]", errors);

  Require(IsPositiveFinite(config.preprocess.voxel_size),
          "downsample.voxel_size must be finite and > 0", errors);
  Require(config.preprocess.crop_to_map.xy_margin >= 0.0f &&
              std::isfinite(config.preprocess.crop_to_map.xy_margin),
          "preprocess.crop_to_map.xy_margin must be finite and >= 0", errors);

  Require(IsUnitInterval(config.obstacle_points_min_evidence) &&
              config.obstacle_points_min_evidence > 0.0f,
          "obstacle_points_min_evidence must be in (0, 1]", errors);
  Require(std::isfinite(config.obstacle_points_min_height) &&
              config.obstacle_points_min_height >= 0.0f,
          "obstacle_points_min_height must be finite and >= 0", errors);
  Require(config.obstacle_points_min_height <= config.geometry.max_step_down,
          "obstacle_points_min_height must be <= max_step_down", errors);
  Require(IsPositiveFinite(config.obstacle_points_max_height_in_base_link),
          "obstacle_points_max_height_in_base_link must be finite and > 0", errors);

  return errors;
}

void ValidateConfigOrThrow(const Config &config) {
  const auto errors = ValidateConfig(config);
  if (errors.empty()) {
    return;
  }
  std::ostringstream message;
  message << "Invalid passable_area config:";
  for (const auto &error : errors) {
    message << " " << error << ";";
  }
  throw std::invalid_argument(message.str());
}

} // namespace passable_area::core
