#ifndef PASSABLE_AREA_CORE_FRAME_PREPROCESSOR_HPP_
#define PASSABLE_AREA_CORE_FRAME_PREPROCESSOR_HPP_

#include "passable_area/core/types/config_types.hpp"
#include "passable_area/core/types/frame_types.hpp"

namespace passable_area::core {

class FramePreprocessor {
public:
  explicit FramePreprocessor(const Config &config) : config_(config) {}

  bool process(const FrameInput &input, ProcessedFrame &output) const;

private:
  Config config_;
};

} // namespace passable_area::core

#endif
