#ifndef PASSABLE_AREA_CORE_MAPPING_LOCAL_TERRAIN_MAP_HPP_
#define PASSABLE_AREA_CORE_MAPPING_LOCAL_TERRAIN_MAP_HPP_

#include "passable_area/core/mapping/terrain_layers.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <Eigen/Core>

#include <vector>

namespace passable_area::core {

class LocalTerrainMap {
public:
  explicit LocalTerrainMap(const Config &config);

  void recenter(const Eigen::Vector2f &base_xy);
  int rows() const { return rows_; }
  int cols() const { return cols_; }
  float resolution() const { return config_.map.resolution; }
  int size() const { return rows_ * cols_; }
  const Eigen::Vector2f &origin() const { return origin_; }
  const Eigen::Vector2f &center() const { return center_; }
  void setFramePeriodSec(float frame_period_sec) { frame_period_sec_ = frame_period_sec; }
  float framePeriodSec() const { return frame_period_sec_; }

  bool worldToIndex(float x, float y, int &index) const;
  Eigen::Vector2f indexToWorld(int index) const;
  TerrainLayers &layers() { return layers_; }
  const TerrainLayers &layers() const { return layers_; }
  void ageCells();

private:
  void initializeLayers();
  void clearLayers();
  void shiftLayers(int row_shift, int col_shift);

  Config config_;
  int rows_ = 0;
  int cols_ = 0;
  Eigen::Vector2f center_ = Eigen::Vector2f::Zero();
  Eigen::Vector2f origin_ = Eigen::Vector2f::Zero();
  float frame_period_sec_ = 0.1f;
  TerrainLayers layers_;
};

} // namespace passable_area::core

#endif
