#ifndef PARAM_UTILS_HPP
#define PARAM_UTILS_HPP

#include "log.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI / 180.0)
#endif

namespace Props {

// -------------------- Types & Enums -------------------- //
enum class Sensors { LIDAR, CAM, INS, TEMP, GPR, NONE, INVALID };
enum class Location { LEFT, RIGHT, CENTRAL, NONE };
enum class TimeScale { SECS, MS, US, NS, M, H };

// -------------------- Location Utils && Sensor Utils -------------------- //
[[maybe_unused]] inline Location parse_location(int code) {
  switch (code) {
  case 0:
    return Location::RIGHT;
  case 1:
    return Location::LEFT;
  case 2:
    return Location::CENTRAL;
  default:
    return Location::NONE;
  }
}

[[maybe_unused]] inline std::string loc2str(Location loc) {
  switch (loc) {
  case Location::LEFT:
    return "LEFT";
  case Location::RIGHT:
    return "RIGHT";
  case Location::CENTRAL:
    return "CENTRAL";
  case Location::NONE:
    return "NONE";
  default:
    return "UNKNOWN";
  }
}

constexpr int sensor_priority(Sensors sensor) {
  switch (sensor) {
  case Sensors::CAM:
  case Sensors::INVALID:
    return 5;
  case Sensors::LIDAR:
    return 4;
  case Sensors::INS:
    return 3;
  case Sensors::TEMP:
  case Sensors::GPR:
    return 1;
  default:
    return 0;
  }
}

constexpr const char *sensors_to_string(const Sensors &sensor) {
  switch (sensor) {
  case Sensors::CAM:
    return "Camera";
  case Sensors::LIDAR:
    return "Lidar";
  case Sensors::TEMP:
    return "Thermometer";
  case Sensors::INS:
    return "Ins";
  case Sensors::GPR:
    return "Gpr";
  default:
    return "None";
  }
}

// -------------------- Math Utils -------------------- //
static inline Eigen::Matrix3d deg_rpy2rot(double roll, double pitch,
                                          double yaw) {
  roll = DEG2RAD(roll);
  pitch = DEG2RAD(pitch);
  yaw = DEG2RAD(yaw);

  Eigen::AngleAxisd r(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd p(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd y(yaw, Eigen::Vector3d::UnitZ());

  // ZYX rotation: yaw -> pitch -> roll
  return (y * p * r).toRotationMatrix();
}

// -------------------- Time Utils -------------------- //
constexpr const char *TIME_FMT = "YYYYMMDDHHMMSS.mmmsss";
using chrono_us = std::chrono::microseconds; // Microseconds precision
using chrono_ms = std::chrono::milliseconds; // Milliseconds precision

[[maybe_unused]] static inline std::string
format_timestamp(double timestamp_sec) {

  // Break into whole seconds and fractional
  std::time_t whole_seconds = static_cast<std::time_t>(timestamp_sec);
  std::tm local_time = *std::localtime(&whole_seconds);

  // Extract fractional microseconds
  auto fractional_us =
      static_cast<int64_t>((timestamp_sec - whole_seconds) * 1'000'000);
  int ms_part = fractional_us / 1'000; // Milliseconds
  int us_part = fractional_us % 1'000; // Microseconds within ms

  // Format timestamp
  std::ostringstream oss;
  oss << std::put_time(&local_time, "%Y%m%d%H%M%S") << "." << std::setw(3)
      << std::setfill('0') << ms_part << std::setw(3) << std::setfill('0')
      << us_part;

  return oss.str();
}

[[maybe_unused]] static inline std::pair<double, std::string>
create_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();

  double curr_time_sec =
      std::chrono::duration_cast<chrono_us>(duration).count() / 1'000'000.0;

  return {curr_time_sec, format_timestamp(curr_time_sec)};
}

[[nodiscard]] inline double time_diff(double newer, double older,
                                      TimeScale scale) {
  double delta_sec = newer - older;

  switch (scale) {
  case TimeScale::SECS:
    return delta_sec;

  case TimeScale::MS:
    return delta_sec * 1'000.0;

  case TimeScale::US:
    return delta_sec * 1'000'000.0;

  case TimeScale::NS:
    return delta_sec * 1'000'000'000.0;

  case TimeScale::M:
    return delta_sec / 60.0;

  case TimeScale::H:
    return delta_sec / 3600.0;

  default:
    return delta_sec; // fallback to seconds
  }
}

[[maybe_unused]] static inline double
bucket_time_start(double timestamp_sec, double comp, TimeScale scale) {
  double multiplier = 1.0;

  switch (scale) {
  case TimeScale::SECS:
    multiplier = 1.0;
    break;
  case TimeScale::MS:
    multiplier = 1e-3;
    break;
  case TimeScale::US:
    multiplier = 1e-6;
    break;
  case TimeScale::NS:
    multiplier = 1e-9;
    break;
  case TimeScale::M:
    multiplier = 60.0;
    break;
  case TimeScale::H:
    multiplier = 3600.0;
    break;
  }

  double interval_sec = comp * multiplier;

  int64_t bucket_idx =
      static_cast<int64_t>(std::floor(timestamp_sec / interval_sec));
  return static_cast<double>(bucket_idx) * interval_sec;
}

template <typename DurationType>
[[maybe_unused]] static inline DurationType
parse_interval(const std::string time_str) {
  static const std::regex time_regex(R"(^(\d+(\.\d+)?)(us|ms|s)$)");
  std::smatch match;

  if (!std::regex_match(time_str, match, time_regex)) {
    throw std::invalid_argument(
        "Invalid time format. Expected format: <value><us|ms|s>");
  }

  double value = std::stod(match[1].str());
  std::string unit = match[3].str();

  if (unit == "us") {
    return std::chrono::duration_cast<DurationType>(
        std::chrono::duration<double, std::micro>(value));
  } else if (unit == "ms") {
    return std::chrono::duration_cast<DurationType>(
        std::chrono::duration<double, std::milli>(value));
  } else if (unit == "s") {
    return std::chrono::duration_cast<DurationType>(
        std::chrono::duration<double>(value));
  } else {
    throw std::invalid_argument("Unsupported time unit.");
  }
}

inline static std::unordered_map<Sensors, chrono_ms> flush_threshold = {
    {Sensors::CAM, chrono_ms(20)},   {Sensors::LIDAR, chrono_ms(40)},
    {Sensors::INS, chrono_ms(60)},   {Sensors::GPR, chrono_ms(120)},
    {Sensors::TEMP, chrono_ms(150)},
};

// -------------------- File Utils -------------------- //
[[maybe_unused]] inline void ensure_dir(const std::string &path) {
  if (!std::filesystem::exists(path))
    std::filesystem::create_directories(path);
}

[[maybe_unused]] inline std::ofstream
write_if_needed(const std::string &filepath, const std::string &header) {
  std::ofstream file;
  if (!std::filesystem::exists(filepath)) {
    file.open(filepath, std::ios::out);
    file << header;
  } else {
    file.open(filepath, std::ios::app);
  }
  return file;
}

[[maybe_unused]] static std::string to_lower(const std::string &str) {
  std::string result = str;
  std::transform(result.begin(), result.end(), result.begin(), ::tolower);
  return result;
}

// -------------------- YAML Utils -------------------- //
template <typename T>
[[maybe_unused]] static inline std::vector<T>
parse_numbers(const YAML::Node &node) {
  std::vector<T> values;
  std::ostringstream msg;

  if (node && node.IsSequence()) { // Ensure it's a list
    for (const auto &item : node) {
      try {
        values.push_back(item.as<T>()); // Automatically convert to int/double
      } catch (const YAML::Exception &e) {

        msg << "Error: Failed to convert value in YAML: " << e.what();
        LOG_ERROR(msg.str());
        std::exit(EXIT_FAILURE);
      }
    }
  } else {
    LOG_ERROR("Error: Expected a YAML sequence but got something else");
    std::exit(EXIT_FAILURE);
  }
  return values;
}

} // namespace Props

// -------- For handling information on sensor location e.t.c -------- //
struct SensorTransform {
  using Map = std::unordered_map<std::string, SensorTransform>;
  using Collection = std::unordered_map<std::string, SensorTransform::Map>;

  std::string name;
  std::string parent_frame;
  Props::Location sensor_location = Props::Location::NONE;
  std::string engineer_notes;

  Eigen::Vector3d translation;
  Eigen::Vector3d rotation_rpy;
  Eigen::Matrix3d axis_remap;
  Eigen::Matrix4d transform;

  bool valid = false;

  SensorTransform();

  static Eigen::Matrix4d invert_transform(const Eigen::Matrix4d &T);

  static SensorTransform load_frm_yaml(const YAML::Node &sensor_node);

  static Eigen::Matrix3d axis_remap_func(const YAML::Node &remap_node);

  static void write_info(const Map &sensors, const std::string &filename);
};

// ------ Sensor Configuration and default values ----- //
namespace SensorConfig {
// initialization of camera parameters
struct Camera {
  bool in_use = false;

  int refresh_delay = 100;      // should be in ms
  int buffer_delay = 5000;      // should be in us
  double exposure = 0.0;        // 0.0 means automatic
  int gain = 0.0;               // 0.0 means automatic
  int refresh_trial = 4;        // number of image refresh attempt
  Props::chrono_ms force_save;  // timed image storage
  int num_buffer = 4;           // buffers per camera
  std::string img_typ = "png";  // image output type
  int quality_compression = 1;  // image quality
  int image_height = 1080;      // 1080 - default image height
  int image_width = 1440;       // 1440 - default image width
  double overlap_ratio = 0.2;   // image similarity ratio
  float size_compression = 0.5; // percentage of output image
  double batch_rate = 1.0;
};

struct Thermometer {
  bool in_use = false;
  double sampling_rate = 0.5; // should be in ms
  Props::chrono_ms sampling_period;
  std::vector<int> port_numbers; // connection channels
  std::string baud_rate = "N/A";
};

struct INS {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool in_use = false;
  std::vector<std::string> urls; // connection channels
  int frequency = 200;           // hz
  int timeout = 100;             // ms
};

struct Lidar {
  bool in_use = false;
  std::vector<std::string> ips;   // Connection ip address
  std::vector<std::string> ports; // Connection ports
  double expected_guage, expected_height_deviation;
};

struct Gpr {
  std::vector<std::string> ips;
  std::vector<std::string> ports;
  int point_per_trace = 250;
  int time_sampling_interval = 200;
  int point_stacks = 4;
  double period_s = 0.00294; // in seconds
  int first_break_point = 0;
  bool in_use = false;
  int output_batch = 100;
};

} // namespace SensorConfig
#endif
