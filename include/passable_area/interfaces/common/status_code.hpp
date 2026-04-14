#ifndef PASSABLE_AREA_INTERFACES_COMMON_STATUS_CODE_HPP_
#define PASSABLE_AREA_INTERFACES_COMMON_STATUS_CODE_HPP_

#include <cstdint>

namespace passable_area::interfaces::common {

enum class StatusCode : int32_t {
  OK_RUNNING = 100,
  ERR_CLOUD_TIMEOUT = 1000,
  ERR_ODOM_TIMEOUT = 1001,
  ERR_SYNC_STALL = 1002,
  ERR_OUTPUT_STALL = 1004,
  FATAL_RUNTIME_EXCEPTION = 1902,
};

inline constexpr int32_t toInt(StatusCode code) {
  return static_cast<int32_t>(code);
}

inline const char *toString(StatusCode code) {
  switch (code) {
  case StatusCode::OK_RUNNING:
    return "OK_RUNNING";
  case StatusCode::ERR_CLOUD_TIMEOUT:
    return "ERR_CLOUD_TIMEOUT";
  case StatusCode::ERR_ODOM_TIMEOUT:
    return "ERR_ODOM_TIMEOUT";
  case StatusCode::ERR_SYNC_STALL:
    return "ERR_SYNC_STALL";
  case StatusCode::ERR_OUTPUT_STALL:
    return "ERR_OUTPUT_STALL";
  case StatusCode::FATAL_RUNTIME_EXCEPTION:
    return "FATAL_RUNTIME_EXCEPTION";
  default:
    return "UNKNOWN_STATUS";
  }
}

} // namespace passable_area::interfaces::common

#endif
