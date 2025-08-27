#include "log.hpp"
#include <spdlog/async_logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// Initializing the logger and parameters for the system
std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;
std::once_flag Logger::init_flag_;

void Logger::initialize(const std::string &log_file_path) {
  std::call_once(init_flag_, [&]() {
    try {
      // Optional but good: Initialize async thread pool early
      spdlog::init_thread_pool(8192, 1); // Queue size, thread count

      // Console Sink (colored)
      auto console_sink =
          std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_pattern("[%Y-%m-%d %H:%M:%S] [%^%l%$] %v");

      // Rotating File Sink: 5MB per file, keep 3 files
      auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
          log_file_path, 1024 * 1024 * 5, 3);
      file_sink->set_pattern("[%Y-%m-%d %H:%M:%S] [%l] %v");

      // Combine sinks
      std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

      // Async logger with combined sinks
      logger_ = std::make_shared<spdlog::async_logger>(
          "multi_sink_logger", sinks.begin(), sinks.end(),
          spdlog::thread_pool(), spdlog::async_overflow_policy::block);

      // Register and set log level
      logger_->set_level(spdlog::level::debug);
      spdlog::register_logger(logger_);

      // Flush behavior
      logger_->flush_on(spdlog::level::err);
      spdlog::flush_every(std::chrono::seconds(3));
    } catch (const spdlog::spdlog_ex &ex) {
      throw std::runtime_error("Log initialization failed: " +
                               std::string(ex.what()));
    }
  });
}

std::shared_ptr<spdlog::logger> Logger::get() {
  if (!logger_) {
    throw std::runtime_error(
        "Logger not initialized! Call Logger::initialize(path) first.");
  }
  return logger_;
}

void Logger::shutdown() {
  if (logger_) {
    spdlog::drop("multi_sink_logger");
    logger_.reset(); // Release memory
  }
}
