#ifndef PASSABLE_AREA_INTERFACES_ROS_RUNTIME_LOGGING_LOG_BRIDGE_HPP_
#define PASSABLE_AREA_INTERFACES_ROS_RUNTIME_LOGGING_LOG_BRIDGE_HPP_

#include <cstdarg>
#include <memory>
#include <string>

namespace passable_area::interfaces::ros {

struct DrLoggerOptions {
  std::string logger_name;
  std::string log_path;
  std::string properties_path;
  bool enable = true;
};

DrLoggerOptions resolveDrLoggerOptions(
    const std::string &default_logger_name,
    const std::string &default_log_path,
    const std::string &default_properties_path);

std::string formatLogMessage(const char *format, va_list args);

class LogBridge {
public:
  explicit LogBridge(DrLoggerOptions options);
  ~LogBridge();

  bool init(std::string &error_message);
  bool isDrEnabled() const;
  bool shouldLogThrottle(const std::string &key, double throttle_sec,
                         double now_sec);

  void debug(const std::string &message) const;
  void info(const std::string &message) const;
  void warn(const std::string &message) const;
  void error(const std::string &message) const;
  void fatal(const std::string &message) const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

} // namespace passable_area::interfaces::ros

#endif
