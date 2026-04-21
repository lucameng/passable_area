#ifndef PASSABLE_AREA_CORE_TYPES_STATE_TYPES_HPP_
#define PASSABLE_AREA_CORE_TYPES_STATE_TYPES_HPP_

#include <cstdint>

namespace passable_area::core {

enum class ObservabilityState : uint8_t {
  kObserved = 0,
  kMissingByDropout = 2,
};

enum class PassabilityState : int8_t {
  kUnknown = -1,
  kPassable = 0,
  kImpassable = 100,
};

enum class SupportState : uint8_t {
  kNone = 0,
  kObserved = 1,
  kPersistent = 2,
};

} // namespace passable_area::core

#endif
