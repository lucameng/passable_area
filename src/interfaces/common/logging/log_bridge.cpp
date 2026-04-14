#include "passable_area/interfaces/common/logging/log_bridge.hpp"

#include <log4cplus_dr/dr_logger.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>

namespace passable_area::interfaces::ros {
namespace {

constexpr const char *kEnvDrLoggerEnable = "PASSABLE_DR_LOGGER_ENABLE";
constexpr const char *kEnvDrLoggerName = "PASSABLE_DR_LOGGER_NAME";
constexpr const char *kEnvDrLoggerPath = "PASSABLE_DR_LOGGER_PATH";
constexpr const char *kEnvDrLoggerPropertiesPath =
    "PASSABLE_DR_LOGGER_PROPERTIES_PATH";

std::once_flag g_dr_logger_initialize_once;

std::string readEnvOrDefault(const char *env_name,
                             const std::string &default_value) {
  const char *env_value = std::getenv(env_name);
  if (!env_value || env_value[0] == '\0') {
    return default_value;
  }
  return std::string(env_value);
}

bool parseEnableEnv(bool default_value) {
  const char *env_value = std::getenv(kEnvDrLoggerEnable);
  if (!env_value || env_value[0] == '\0') {
    return default_value;
  }

  std::string lowered(env_value);
  std::transform(
      lowered.begin(), lowered.end(), lowered.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (lowered == "0" || lowered == "false" || lowered == "off" ||
      lowered == "no") {
    return false;
  }
  if (lowered == "1" || lowered == "true" || lowered == "on" ||
      lowered == "yes") {
    return true;
  }
  return default_value;
}

template <typename LogFn>
void safeDrLogCall(const std::shared_ptr<DrLogger> &logger, LogFn &&log_fn) {
  if (!logger) {
    return;
  }
  try {
    log_fn(*logger);
  } catch (...) {
    // Logging backend failure must not affect node runtime.
  }
}

} // namespace

struct LogBridge::Impl {
  explicit Impl(DrLoggerOptions in_options) : options(std::move(in_options)) {}

  DrLoggerOptions options;
  std::shared_ptr<DrLogger> logger;
  bool dr_enabled = false;

  std::mutex throttle_mutex;
  std::unordered_map<std::string, double> last_warn_time_sec;
};

DrLoggerOptions
resolveDrLoggerOptions(const std::string &default_logger_name,
                       const std::string &default_log_path,
                       const std::string &default_properties_path) {
  DrLoggerOptions options;
  options.enable = parseEnableEnv(true);
  options.logger_name = readEnvOrDefault(kEnvDrLoggerName, default_logger_name);
  options.log_path = readEnvOrDefault(kEnvDrLoggerPath, default_log_path);
  options.properties_path =
      readEnvOrDefault(kEnvDrLoggerPropertiesPath, default_properties_path);
  return options;
}

std::string formatLogMessage(const char *format, va_list args) {
  if (format == nullptr) {
    return std::string("<null log format>");
  }

  va_list args_copy;
  va_copy(args_copy, args);
  const int size = std::vsnprintf(nullptr, 0, format, args_copy);
  va_end(args_copy);

  if (size <= 0) {
    return std::string(format);
  }

  std::string buffer(static_cast<size_t>(size) + 1, '\0');
  std::vsnprintf(buffer.data(), buffer.size(), format, args);
  buffer.resize(static_cast<size_t>(size));
  return buffer;
}

LogBridge::LogBridge(DrLoggerOptions options)
    : impl_(std::make_shared<Impl>(std::move(options))) {}

LogBridge::~LogBridge() = default;

bool LogBridge::init(std::string &error_message) {
  error_message.clear();
  if (!impl_->options.enable) {
    impl_->dr_enabled = false;
    return false;
  }

  if (impl_->options.log_path.empty()) {
    impl_->dr_enabled = false;
    error_message = "DrLogger log path is empty";
    return false;
  }

  const std::filesystem::path log_path(impl_->options.log_path);
  std::error_code fs_error;
  const bool log_path_exists = std::filesystem::exists(log_path, fs_error);
  if (fs_error || !log_path_exists ||
      !std::filesystem::is_directory(log_path, fs_error)) {
    impl_->dr_enabled = false;
    error_message = "DrLogger log path not found: " + impl_->options.log_path;
    return false;
  }

  if (impl_->options.properties_path.empty()) {
    impl_->dr_enabled = false;
    error_message = "DrLogger properties path is empty";
    return false;
  }

  const std::filesystem::path properties_path(impl_->options.properties_path);
  fs_error.clear();
  const bool properties_exists =
      std::filesystem::exists(properties_path, fs_error);
  if (fs_error || !properties_exists ||
      !std::filesystem::is_regular_file(properties_path, fs_error)) {
    impl_->dr_enabled = false;
    error_message =
        "DrLogger properties file not found: " + impl_->options.properties_path;
    return false;
  }

  try {
    const auto options = impl_->options;
    std::call_once(g_dr_logger_initialize_once, [options]() {
      DrLogger::initialize(options.logger_name, options.log_path,
                           options.properties_path);
    });

    impl_->logger = std::make_shared<DrLogger>(impl_->options.logger_name);
    impl_->dr_enabled = static_cast<bool>(impl_->logger);
    return impl_->dr_enabled;
  } catch (const std::exception &ex) {
    impl_->dr_enabled = false;
    error_message = ex.what();
    return false;
  } catch (...) {
    impl_->dr_enabled = false;
    error_message = "unknown exception";
    return false;
  }
}

bool LogBridge::isDrEnabled() const { return impl_->dr_enabled; }

bool LogBridge::shouldLogThrottle(const std::string &key, double throttle_sec,
                                  double now_sec) {
  if (throttle_sec <= 0.0 || key.empty() || !std::isfinite(now_sec)) {
    return true;
  }

  std::lock_guard<std::mutex> lk(impl_->throttle_mutex);
  const auto iter = impl_->last_warn_time_sec.find(key);
  if (iter != impl_->last_warn_time_sec.end()) {
    const double elapsed_sec = now_sec - iter->second;
    if (elapsed_sec < 0.0) {
      impl_->last_warn_time_sec[key] = now_sec;
      return true;
    }
    if (elapsed_sec < throttle_sec) {
      return false;
    }
  }
  impl_->last_warn_time_sec[key] = now_sec;
  return true;
}

void LogBridge::debug(const std::string &message) const {
  if (!impl_->dr_enabled) {
    return;
  }
  safeDrLogCall(impl_->logger,
                [&](DrLogger &logger) { logger.debug("%s", message.c_str()); });
}

void LogBridge::info(const std::string &message) const {
  if (!impl_->dr_enabled) {
    return;
  }
  safeDrLogCall(impl_->logger,
                [&](DrLogger &logger) { logger.info("%s", message.c_str()); });
}

void LogBridge::warn(const std::string &message) const {
  if (!impl_->dr_enabled) {
    return;
  }
  safeDrLogCall(impl_->logger,
                [&](DrLogger &logger) { logger.warn("%s", message.c_str()); });
}

void LogBridge::error(const std::string &message) const {
  if (!impl_->dr_enabled) {
    return;
  }
  safeDrLogCall(impl_->logger,
                [&](DrLogger &logger) { logger.error("%s", message.c_str()); });
}

void LogBridge::fatal(const std::string &message) const {
  if (!impl_->dr_enabled) {
    return;
  }
  safeDrLogCall(impl_->logger,
                [&](DrLogger &logger) { logger.fatal("%s", message.c_str()); });
}

} // namespace passable_area::interfaces::ros
