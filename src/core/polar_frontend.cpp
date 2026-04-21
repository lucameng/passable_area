#include "passable_area/core/polar_frontend.hpp"
#include "passable_area/core/utils/math_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace passable_area::core {
namespace {

struct CellStats {
  float min_z = std::numeric_limits<float>::infinity();
  float max_z = -std::numeric_limits<float>::infinity();
  std::vector<float> samples_z;
  int count = 0;
};

struct HeightBand {
  float bottom = std::numeric_limits<float>::quiet_NaN();
  float top = std::numeric_limits<float>::quiet_NaN();
  int count = 0;
};

float NormalizeAngle(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

void AccumulateCellStats(CellStats &stats, float z) {
  stats.min_z = std::min(stats.min_z, z);
  stats.max_z = std::max(stats.max_z, z);
  stats.samples_z.push_back(z);
  ++stats.count;
}

int CountActiveCells(const std::vector<CellStats> &stats_by_cell) {
  return static_cast<int>(
      std::count_if(stats_by_cell.begin(), stats_by_cell.end(),
                    [](const auto &stats) { return stats.count > 0; }));
}

float Quantile(const std::vector<float> &values, int begin, int end, float q) {
  const int count = end - begin;
  if (count <= 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const int offset = std::clamp(
      static_cast<int>(std::floor(static_cast<float>(count - 1) * q)), 0,
      count - 1);
  return values[static_cast<size_t>(begin + offset)];
}

std::vector<HeightBand> SplitHeightBands(std::vector<float> values,
                                         float split_gap) {
  std::vector<HeightBand> bands;
  if (values.empty()) {
    return bands;
  }
  std::sort(values.begin(), values.end());
  int begin = 0;
  const auto flush = [&](int end) {
    HeightBand band;
    band.bottom = Quantile(values, begin, end, 0.10f);
    band.top = Quantile(values, begin, end, 0.90f);
    band.count = end - begin;
    bands.push_back(band);
  };
  for (int i = 1; i < static_cast<int>(values.size()); ++i) {
    if (values[static_cast<size_t>(i)] - values[static_cast<size_t>(i - 1)] >
        split_gap) {
      flush(i);
      begin = i;
    }
  }
  flush(static_cast<int>(values.size()));
  return bands;
}

} // namespace

bool MapGeometry::mapToIndex(float x, float y, int &index) const {
  const int col = static_cast<int>(std::floor((x - origin.x()) / resolution));
  const int row = static_cast<int>(std::floor((y - origin.y()) / resolution));
  if (row < 0 || row >= rows || col < 0 || col >= cols) {
    return false;
  }
  index = row * cols + col;
  return true;
}

Eigen::Vector2f MapGeometry::indexToMap(int index) const {
  const int row = index / cols;
  const int col = index % cols;
  return Eigen::Vector2f(
      origin.x() + (static_cast<float>(col) + 0.5f) * resolution,
      origin.y() + (static_cast<float>(row) + 0.5f) * resolution);
}

FrontendOutput PolarFrontend::run(const ProcessedFrame &frame,
                                  const FrameObservability &observability,
                                  const MapGeometry &geo) const {
  FrontendOutput output;
  const size_t cell_count = static_cast<size_t>(geo.size);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  output.support_anchor_used.assign(cell_count, nan);
  output.support_anchor_origin.assign(
      cell_count, static_cast<uint8_t>(SupportAnchorOrigin::kNone));
  output.support_anchor_authority.assign(
      cell_count, static_cast<uint8_t>(SupportAnchorAuthority::kInvalid));
  output.anchor_leak_suppression_enabled.assign(cell_count, 0U);
  output.sub_support_leak_count.assign(cell_count, 0U);
  output.anchor_below_observation_count.assign(cell_count, 0U);
  output.stale_anchor_residual_filtered_count.assign(cell_count, 0U);
  output.raw_sample_min_z.assign(cell_count, nan);
  output.raw_sample_max_z.assign(cell_count, nan);
  output.raw_sample_count.assign(cell_count, 0U);
  output.filtered_sample_min_z.assign(cell_count, nan);
  output.filtered_sample_max_z.assign(cell_count, nan);
  output.filtered_sample_count.assign(cell_count, 0U);
  output.raw_upper_support_cell.assign(cell_count, 0U);
  output.explanation_adjusted_upper_support_cell.assign(cell_count, 0U);
  output.upper_support_cell.assign(cell_count, 0U);
  output.obstacle_local_triggered.assign(cell_count, 0U);
  output.obstacle_upper_patch_confirmed.assign(cell_count, 0U);
  output.obstacle_explanation_rejected.assign(cell_count, 0U);
  output.obstacle_suspicious.assign(cell_count, 0U);
  output.obstacle_candidate_cell.assign(cell_count, 0U);
  output.obstacle_rejected_by_neighbor_support.assign(cell_count, 0U);
  output.neighbor_upper_support_count.assign(cell_count, 0);
  output.aligned_neighbor_support_count.assign(cell_count, 0);
  output.explanation_decision.assign(
      cell_count, static_cast<uint8_t>(FrontendExplanationDecision::kNone));
  output.facade_lower_upper_coexisting.assign(cell_count, 0U);
  output.facade_upper_edge_aligned_with_supported_neighbors.assign(cell_count,
                                                                   0U);

  std::vector<CellStats> stats_by_cell(cell_count);
  for (const auto &sample : frame.map_samples) {
    int cell = -1;
    if (!geo.mapToIndex(sample.point_in_map.x, sample.point_in_map.y, cell)) {
      continue;
    }
    AccumulateCellStats(stats_by_cell[static_cast<size_t>(cell)],
                        sample.point_in_map.z);
  }

  for (size_t cell = 0; cell < cell_count; ++cell) {
    const auto &stats = stats_by_cell[cell];
    if (stats.count == 0) {
      continue;
    }
    output.raw_sample_min_z[cell] = stats.min_z;
    output.raw_sample_max_z[cell] = stats.max_z;
    output.raw_sample_count[cell] = static_cast<uint16_t>(
        std::clamp(stats.count, 0,
                   static_cast<int>(std::numeric_limits<uint16_t>::max())));
    output.filtered_sample_min_z[cell] = stats.min_z;
    output.filtered_sample_max_z[cell] = stats.max_z;
    output.filtered_sample_count[cell] = output.raw_sample_count[cell];
  }

  const int active_cell_count = CountActiveCells(stats_by_cell);
  output.support_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.protrusion_candidates.reserve(static_cast<size_t>(active_cell_count));
  output.overhead_candidates.reserve(static_cast<size_t>(active_cell_count));

  const float suspicious_vertical_span = config_.geometry.max_step_up * 0.75f;
  const float split_gap = std::max(config_.geometry.profile_split_gap, 1e-3f);
  const float yaw = YawFromQuaternion(frame.base_pose_in_map.orientation);
  const size_t sector_count =
      std::max<size_t>(observability.sectors.size(), static_cast<size_t>(1));
  const float sector_size =
      2.0f * static_cast<float>(M_PI) / static_cast<float>(sector_count);

  for (int cell = 0; cell < geo.size; ++cell) {
    const auto &stats = stats_by_cell[static_cast<size_t>(cell)];
    if (stats.count == 0) {
      continue;
    }

    ObservabilityState sector_state = ObservabilityState::kObserved;
    float coverage = 0.0f;
    if (!observability.sectors.empty()) {
      const Eigen::Vector2f center = geo.indexToMap(cell);
      const float representative_base_angle = NormalizeAngle(
          std::atan2(center.y() - frame.base_pose_in_map.position.y(),
                     center.x() - frame.base_pose_in_map.position.x()) -
          yaw);
      const int sector =
          std::clamp(static_cast<int>(std::floor((representative_base_angle +
                                                  static_cast<float>(M_PI)) /
                                                 sector_size)),
                     0, static_cast<int>(observability.sectors.size()) - 1);
      sector_state = observability.sectors[static_cast<size_t>(sector)].state;
      coverage = observability.sectors[static_cast<size_t>(sector)]
                     .coverage_confidence;
    }

    const auto bands = SplitHeightBands(stats.samples_z, split_gap);
    if (bands.empty()) {
      continue;
    }
    size_t support_band_index = 0U;
    if (bands.size() > 1U && bands[0].count == 1 && bands[1].count >= 3) {
      support_band_index = 1U;
    }
    const HeightBand &support_band = bands[support_band_index];
    const float support_z = support_band.bottom;

    const float vertical_span = stats.max_z - stats.min_z;
    const bool dropout = sector_state == ObservabilityState::kMissingByDropout;
    if (!dropout) {
      output.support_candidates.push_back(
          SupportCandidate{cell, support_z, std::clamp(coverage, 0.0f, 1.0f)});
    }

    const bool has_upper_band = support_band_index + 1U < bands.size();
    const HeightBand &protrusion_band =
        has_upper_band ? bands.back() : support_band;
    const HeightBand *overhead_band = nullptr;
    for (size_t band_index = support_band_index + 1U; band_index < bands.size();
         ++band_index) {
      if (bands[band_index].bottom - support_z <
          config_.geometry.min_clearance) {
        overhead_band = &bands[band_index];
        break;
      }
    }
    const float obstacle_z = has_upper_band ? protrusion_band.top : stats.max_z;
    const float height_above_support = obstacle_z - support_z;
    const bool protrusion_triggered =
        height_above_support > suspicious_vertical_span;
    const bool overhead_triggered = overhead_band != nullptr;

    if (protrusion_triggered || overhead_triggered) {
      output.obstacle_local_triggered[static_cast<size_t>(cell)] = 1U;
      output.obstacle_upper_patch_confirmed[static_cast<size_t>(cell)] = 1U;
      output.obstacle_suspicious[static_cast<size_t>(cell)] = 1U;
      output.obstacle_candidate_cell[static_cast<size_t>(cell)] = 1U;
    }
    if (protrusion_triggered) {
      output.protrusion_candidates.push_back(ProtrusionCandidate{
          cell, obstacle_z, std::clamp(height_above_support, 0.0f, 1.0f),
          1.0f});
    }
    if (overhead_triggered) {
      output.overhead_candidates.push_back(
          OverheadCandidate{cell, overhead_band->bottom,
                            std::clamp(config_.geometry.min_clearance -
                                           (overhead_band->bottom - support_z),
                                       0.0f, 1.0f),
                            1.0f});
    }
  }

  return output;
}

} // namespace passable_area::core
