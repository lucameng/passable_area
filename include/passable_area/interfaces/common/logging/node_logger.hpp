#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_LOGGING_NODE_LOGGER_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_LOGGING_NODE_LOGGER_HPP_

#include "passable_area/interfaces/common/logging/log_bridge.hpp"

#include <cstdarg>
#include <functional>
#include <memory>
#include <string>

namespace passable_area::interfaces::ros {

class NodeLogger {
public:
  enum class Level {
    kDebug = 0,
    kInfo,
    kWarn,
    kError,
    kFatal,
  };

  using RosLogFn = std::function<void(Level level, const std::string &message)>;
  using NowSecFn = std::function<double()>;

  NodeLogger(std::shared_ptr<LogBridge> log_bridge, RosLogFn ros_log_fn,
             NowSecFn now_sec_fn);

  void log(Level level, const char *format, ...);
  void logV(Level level, const char *format, va_list args);
  void logWarnThrottle(double throttle_sec, const char *key,
                       const char *format, ...);
  void logWarnThrottleV(double throttle_sec, const char *key,
                        const char *format, va_list args);

private:
  void emit(Level level, const std::string &message);

  std::shared_ptr<LogBridge> log_bridge_;
  RosLogFn ros_log_fn_;
  NowSecFn now_sec_fn_;
};

} // namespace passable_area::interfaces::ros

#endif
