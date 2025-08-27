#ifndef ATIS_PARAMS_HPP
#define ATIS_PARAMS_HPP
#include "log.hpp"
#include "utils.hpp"

struct SensorParams {
  using TransformInfo = std::unordered_map<std::string, Eigen::Matrix4d>;

  SensorConfig::Camera cam;
  SensorConfig::Lidar lidar;
  SensorConfig::Thermometer temp;
  SensorConfig::INS ins;
  SensorConfig::Gpr gpr;

  std::string log_folder, spot_check_log, acq_track_log;
  std::string data_storage, general_storage;

  double min_speed_for_acq = 0.0, data_batch_time = 0.0;
  double init_complete_time = 0.0;
  int pool_threads = 4;

  bool create_folders = true;

  std::unordered_map<Props::Sensors, TransformInfo> sensor_pose;
};

// place holder for all parameters
extern SensorParams params;

extern SensorTransform::Collection transform_info;

// parameter initializer
struct InitParameters {
  // helps to intialize classes
  static void init_params(SensorParams &params, const std::string &launch_path);

  static void initialize_general(YAML::Node &config, SensorParams &params);

  static void initialize_camera(YAML::Node &config, SensorParams &params);

  static void initialize_thermometer(YAML::Node &config, SensorParams &params);

  static void initialize_ins(YAML::Node &config, SensorParams &params);

  static void initialize_lidar(YAML::Node &config, SensorParams &params);

  static void initialize_gpr(YAML::Node &config, SensorParams &params);

  static void initialize_log_params_alt(const std::string &path);

private:
  static std::unordered_map<std::string,
                            std::function<void(YAML::Node &, SensorParams &)>>
      sensor_init;
};
#endif
