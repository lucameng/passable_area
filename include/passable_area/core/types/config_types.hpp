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
  float profile_split_gap = 0.10f;
};

struct ObservabilityConfig {
  int sector_count = 72;
  int min_points_per_sector = 12;
  int dropout_sector_gap_threshold = 8;
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
  float obstacle_clear_observed_decay = 0.20f;
  float obstacle_clear_partial_decay_scale = 0.35f;
  float obstacle_height_clear_threshold = 0.25f;
};

struct PreprocessConfig {
  struct BodyFilterConfig {
    bool enable = false;
    float x_min = -0.4f;
    float x_max = 0.4f;
    float y_min = -0.2f;
    float y_max = 0.2f;
    float z_min = -0.5f;
    float z_max = 0.5f;
  };

  struct CropToMapConfig {
    bool enable = true;
    float xy_margin = 0.0f;
  };

  float voxel_size = 0.08f;
  bool enable_downsample = true;
  BodyFilterConfig body_filter;
  CropToMapConfig crop_to_map;
};

struct DebugConfig {
  bool publish_grid_map = true;
  bool publish_points = true;
  bool publish_base_gravity_cloud = false;
  bool publish_observability = true;
  bool publish_map_to_base_gravity_tf = false;
};

struct Config {
  MapConfig map;
  GeometryConfig geometry;
  ObservabilityConfig observability;
  PersistenceConfig persistence;
  PreprocessConfig preprocess;
  DebugConfig debug;
  float obstacle_points_min_evidence = 0.4f;
  float obstacle_points_min_height = 0.20f;
  float obstacle_points_max_height_in_base_link = 0.20f;
  std::string map_frame = "map";
  std::string base_gravity_frame = "base_gravity";
  std::string body_frame = "base_link";
};

} // namespace passable_area::core

#endif
