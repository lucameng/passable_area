#include "passable_area/core/mapping/local_terrain_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace passable_area::core {
namespace {

template <typename T>
void Fill(std::vector<T> &values, const T &value, int count) {
  values.assign(count, value);
}

} // namespace

LocalTerrainMap::LocalTerrainMap(const Config &config) : config_(config) {
  cols_ = std::max(1, static_cast<int>(std::round(config_.map.length / config_.map.resolution)));
  rows_ = std::max(1, static_cast<int>(std::round(config_.map.width / config_.map.resolution)));
  initializeLayers();
  recenter(Eigen::Vector2f::Zero());
}

void LocalTerrainMap::initializeLayers() {
  const int cell_count = size();
  Fill(layers_.support_height, std::numeric_limits<float>::quiet_NaN(), cell_count);
  Fill(layers_.support_confidence, 0.0f, cell_count);
  Fill(layers_.overhead_height, std::numeric_limits<float>::quiet_NaN(), cell_count);
  Fill(layers_.overhead_confidence, 0.0f, cell_count);
  Fill(layers_.obstacle_evidence, 0.0f, cell_count);
  Fill(layers_.coverage_confidence, 0.0f, cell_count);
  Fill(layers_.slope, 0.0f, cell_count);
  Fill(layers_.step_up, 0.0f, cell_count);
  Fill(layers_.step_down, 0.0f, cell_count);
  Fill(layers_.roughness, 0.0f, cell_count);
  Fill(layers_.clearance, std::numeric_limits<float>::quiet_NaN(), cell_count);
  Fill(layers_.support_continuity, 0.0f, cell_count);
  Fill(layers_.support_state, static_cast<uint8_t>(SupportState::kNone), cell_count);
  Fill(layers_.passability_state, static_cast<int8_t>(PassabilityState::kUnknown), cell_count);
  Fill(layers_.traversal_cost, static_cast<int8_t>(-1), cell_count);
  Fill(layers_.last_sector_state, static_cast<uint8_t>(ObservabilityState::kPartiallyObserved),
       cell_count);
  Fill(layers_.last_observed_age, static_cast<uint16_t>(0), cell_count);
  Fill(layers_.last_reliable_age, static_cast<uint16_t>(0), cell_count);
}

void LocalTerrainMap::recenter(const Eigen::Vector2f &base_xy) {
  center_ = base_xy;
  origin_.x() = center_.x() - 0.5f * config_.map.length;
  origin_.y() = center_.y() - 0.5f * config_.map.width;
}

bool LocalTerrainMap::worldToIndex(float x, float y, int &index) const {
  const int col = static_cast<int>(std::floor((x - origin_.x()) / config_.map.resolution));
  const int row = static_cast<int>(std::floor((y - origin_.y()) / config_.map.resolution));
  if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
    return false;
  }
  index = row * cols_ + col;
  return true;
}

Eigen::Vector2f LocalTerrainMap::indexToWorld(int index) const {
  const int row = index / cols_;
  const int col = index % cols_;
  return Eigen::Vector2f(origin_.x() + (static_cast<float>(col) + 0.5f) * config_.map.resolution,
                         origin_.y() + (static_cast<float>(row) + 0.5f) * config_.map.resolution);
}

void LocalTerrainMap::ageCells() {
  for (size_t i = 0; i < layers_.last_observed_age.size(); ++i) {
    layers_.last_observed_age[i] = std::min<uint16_t>(layers_.last_observed_age[i] + 1U, 60000U);
    layers_.last_reliable_age[i] = std::min<uint16_t>(layers_.last_reliable_age[i] + 1U, 60000U);
  }
}

} // namespace passable_area::core
