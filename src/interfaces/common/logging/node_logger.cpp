#include "passable_area/interfaces/common/logging/node_logger.hpp"

#include <limits>
#include <string>
#include <utility>

namespace passable_area::interfaces::ros {

NodeLogger::NodeLogger(std::shared_ptr<LogBridge> log_bridge,
                       RosLogFn ros_log_fn, NowSecFn now_sec_fn)
    : log_bridge_(std::move(log_bridge)), ros_log_fn_(std::move(ros_log_fn)),
      now_sec_fn_(std::move(now_sec_fn)) {}

void NodeLogger::log(Level level, const char *format, ...) {
  va_list args;
  va_start(args, format);
  logV(level, format, args);
  va_end(args);
}

void NodeLogger::logV(Level level, const char *format, va_list args) {
  const std::string message = formatLogMessage(format, args);
  emit(level, message);
}

void NodeLogger::logWarnThrottle(double throttle_sec, const char *key,
                                 const char *format, ...) {
  va_list args;
  va_start(args, format);
  logWarnThrottleV(throttle_sec, key, format, args);
  va_end(args);
}

void NodeLogger::logWarnThrottleV(double throttle_sec, const char *key,
                                  const char *format, va_list args) {
  const std::string message = formatLogMessage(format, args);

  if (log_bridge_ && throttle_sec > 0.0) {
    const std::string throttle_key = key != nullptr ? key : "";
    const double now_sec =
        now_sec_fn_ ? now_sec_fn_() : std::numeric_limits<double>::quiet_NaN();
    if (!log_bridge_->shouldLogThrottle(throttle_key, throttle_sec, now_sec)) {
      return;
    }
  }

  emit(Level::kWarn, message);
}

void NodeLogger::emit(Level level, const std::string &message) {
  const bool dr_enabled = log_bridge_ && log_bridge_->isDrEnabled();
  if (dr_enabled) {
    switch (level) {
    case Level::kDebug:
      log_bridge_->debug(message);
      break;
    case Level::kInfo:
      log_bridge_->info(message);
      break;
    case Level::kWarn:
      log_bridge_->warn(message);
      break;
    case Level::kError:
      log_bridge_->error(message);
      break;
    case Level::kFatal:
      log_bridge_->fatal(message);
      break;
    }
    return;
  }

  if (ros_log_fn_) {
    ros_log_fn_(level, message);
  }
}

} // namespace passable_area::interfaces::ros
