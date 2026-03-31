#ifndef PASSABLE_AREA_CORE_TYPES_CONFIG_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_CONFIG_TYPES_HPP_

#include <string>

namespace passable_area::core {

struct MapConfig {
  float length = 8.0f;
  float width = 8.0f;
  float resolution = 0.1f;
  float height_min = -1.0f;
  float height_max = 1.0f;
};

struct GeometryConfig {
  float max_support_slope_deg = 25.0f;
  float max_step_up = 0.2f;
  float max_step_down = 0.25f;
  float max_support_roughness = 0.08f;
  float min_clearance = 0.35f;
};

struct ObservabilityConfig {
  int sector_count = 72;
  int min_points_per_sector = 12;
  int dropout_sector_gap_threshold = 8;
  float blind_rear_half_width_deg = 25.0f;
  float partial_sector_ratio = 0.35f;
  float stale_to_unknown_time = 0.8f;
  float min_support_confidence = 0.2f;
};

struct PersistenceConfig {
  int support_persistence_frames = 5;
  float support_confidence_gain = 0.35f;
  float support_confidence_decay = 0.08f;
  float obstacle_evidence_gain = 0.25f;
  float obstacle_evidence_decay = 0.05f;
};

struct PreprocessConfig {
  float voxel_size = 0.08f;
  bool enable_downsample = true;
};

struct DebugConfig {
  bool publish_grid_map = true;
  bool publish_points = true;
  bool publish_observability = true;
};

struct Config {
  MapConfig map;
  GeometryConfig geometry;
  ObservabilityConfig observability;
  PersistenceConfig persistence;
  PreprocessConfig preprocess;
  DebugConfig debug;
  std::string world_frame = "camera_init";
  std::string gravity_frame = "base_gravity";
  std::string body_frame = "base_link";
};

} // namespace passable_area::core

#endif
