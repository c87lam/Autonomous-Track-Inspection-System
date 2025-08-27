#ifndef OPERATION_LOGGER_HPP
#define OPERATION_LOGGER_HPP

#include <memory>
#include <mutex>
#include <spdlog/async.h>
#include <spdlog/spdlog.h>

// Compile-time log level filter (strip logs in Release if needed)
#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif

class Logger {
public:
  // Initialize with path to log file
  static void initialize(const std::string &log_file_path);

  // Get the shared logger instance
  static std::shared_ptr<spdlog::logger> get();

  // Drop and clean up logger
  static void shutdown();

private:
  static std::shared_ptr<spdlog::logger> logger_;
  static std::once_flag init_flag_;
};

// Logging Macros
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(Logger::get(), __VA_ARGS__)
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(Logger::get(), __VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(Logger::get(), __VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(Logger::get(), __VA_ARGS__)

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(Logger::get(), __VA_ARGS__)
#else
#define LOG_DEBUG(...) ((void)0)
#endif
#endif
