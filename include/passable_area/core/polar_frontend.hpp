#ifndef PASSABLE_AREA_CORE_POLAR_FRONTEND_HPP_
#define PASSABLE_AREA_CORE_POLAR_FRONTEND_HPP_

#include "passable_area/core/candidate_types.hpp"
#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

#include <Eigen/Core>

namespace passable_area::core {

struct MapGeometry {
  int rows = 0;
  int cols = 0;
  int size = 0;
  float resolution = 0.1f;
  Eigen::Vector2f origin = Eigen::Vector2f::Zero();

  bool mapToIndex(float x, float y, int &index) const;
  Eigen::Vector2f indexToMap(int index) const;
};

class PolarFrontend {
public:
  explicit PolarFrontend(const Config &config) : config_(config) {}

  FrontendOutput run(const ProcessedFrame &frame,
                     const FrameObservability &observability,
                     const MapGeometry &geo) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
