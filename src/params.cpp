#include "param.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

SensorParams params;
SensorTransform::Collection transform_info;

// ------ MAPPING STRING TO FUNCTIONS ------------------- //

struct SensorKeys {
  static constexpr const char *General = "general";
  static constexpr const char *Camera = "camera";
  static constexpr const char *Thermometer = "thermometer";
  static constexpr const char *Lidar = "lidar";
  static constexpr const char *Ins = "ins";
  static constexpr const char *Gpr = "gpr";
};

std::unordered_map<std::string,
                   std::function<void(YAML::Node &, SensorParams &)>>
    InitParameters::sensor_init = {
        {SensorKeys::Camera, InitParameters::initialize_camera},
        {SensorKeys::Thermometer, InitParameters::initialize_thermometer},
        {SensorKeys::Ins, InitParameters::initialize_ins},
        {SensorKeys::Lidar, InitParameters::initialize_lidar},
        {SensorKeys::Gpr, InitParameters::initialize_gpr}};

void InitParameters::initialize_general(YAML::Node &config,
                                        SensorParams &params) {
  const auto &general = config["general"];

  try {
    // Access the logging configuration
    params.log_folder = general["log_folder"].as<std::string>("./log");
    Props::ensure_dir(params.log_folder);

    params.data_storage = general["storage_folder"].as<std::string>("./data");
    Props::ensure_dir(params.data_storage);

    // minimum speed to start data acquisition
    params.min_speed_for_acq = general["min_speed_for_start"].as<double>(0.2);
    auto data_batch_time = general["data_batch_time"].as<std::string>("1s");
    params.data_batch_time =
        Props::parse_interval<std::chrono::seconds>(data_batch_time).count();

    if (params.create_folders) {
      std::filesystem::path base = params.log_folder;
      params.general_storage = base.string();

      std::string msg = "atis_log.log";
      std::filesystem::path atis_log_path = base / msg;
      Logger::initialize(atis_log_path.string());

      msg = "spot_check_log.txt";
      std::filesystem::path scl = base / msg;
      params.spot_check_log = scl.string();

      msg = "acqusition_track.txt";
      scl = base / msg;
      params.acq_track_log = scl.string();
    }

  } catch (const YAML::Exception &e) {
    std::cerr << "Error Initializing General Settings" << e.what() << "\n";
  }
}

void InitParameters::initialize_camera(YAML::Node &config,
                                       SensorParams &params) {
  LOG_INFO("Establishing Camera Parameters.");

  try {
    std::string val;

    // Access the camera configurations
    auto &camera = params.cam;
    camera.in_use = true;

    val = config["refresh_delay"].as<std::string>("100ms");
    camera.refresh_delay =
        Props::parse_interval<std::chrono::milliseconds>(val).count();

    val = config["buffer_delay"].as<std::string>("1s");
    camera.buffer_delay =
        Props::parse_interval<std::chrono::milliseconds>(val).count();

    val = config["exposure_time"].as<std::string>("1s");
    camera.exposure =
        Props::parse_interval<std::chrono::microseconds>(val).count();

    val = config["force_save"].as<std::string>("1s");
    camera.force_save = Props::parse_interval<std::chrono::milliseconds>(val);

    camera.refresh_trial = config["refresh_trial"].as<int>(3);
    camera.gain = config["gain"].as<double>(0.0);
    camera.image_height = config["acqusition_image_height"].as<int>(1080);
    camera.image_width = config["acqusition_image_width"].as<double>(1440);

    camera.size_compression = config["size_compression"].as<double>(0.5);
    if (camera.size_compression < 0.0f || camera.size_compression > 1.0f)
      camera.size_compression = 0.5;

    // Determine the image type
    camera.img_typ = config["img_type"].as<std::string>("jpeg");
    camera.img_typ = Props::to_lower(camera.img_typ);

    // Compression ratio (validate range 1-10)
    int comp_ratio = config["quality_compression"].as<int>(5);
    camera.quality_compression =
        (comp_ratio >= 1 && comp_ratio <= 10) ? comp_ratio : 5;

    // Buffer size (default: 2 if invalid or missing)
    int num_buffer = config["num_buffer_per_camera"].as<int>(2);
    camera.num_buffer = num_buffer > 0 ? num_buffer : 2;

    // For pruning - maximum overlap allowed
    camera.overlap_ratio = config["overlap_ratio"].as<double>(0.5);
    if (camera.overlap_ratio < 0.0 || camera.overlap_ratio > 1.0)
      camera.overlap_ratio = 0.8;

  } catch (const YAML::Exception &e) {
    LOG_ERROR("Error in Establishing Camera Parameters");
    std::exit(0);
  }
}

void InitParameters::initialize_thermometer(YAML::Node &config,
                                            SensorParams &params) {
  LOG_INFO("Establishing Thermometer Parameters.");

  try {
    auto &temp = params.temp;
    temp.in_use = true;

    std::string val = config["sampling_rate"].as<std::string>("0.5s");
    temp.sampling_period =
        Props::parse_interval<std::chrono::milliseconds>(val);
    temp.sampling_rate =
        std::chrono::duration<double>(temp.sampling_period).count();

    temp.port_numbers = Props::parse_numbers<int>(config["port_number"]);

    int b_rate = config["baud_rate"].as<int>(115200);
    temp.baud_rate = "B" + std::to_string(b_rate);

  } catch (const YAML::Exception &e) {
    std::cout << e.what() << std::endl;
    LOG_ERROR("Error in Establishing Thermometer Parameters");
    std::exit(0);
  }
}

void InitParameters::initialize_ins(YAML::Node &config, SensorParams &params) {
  LOG_INFO("Establishing Ins Parameters.");

  try {
    auto &ins = params.ins;
    ins.in_use = true;
    ins.urls = Props::parse_numbers<std::string>(config["device_url"]);

    ins.frequency = config["timeout"].as<double>(200);

    std::string val = config["timeout"].as<std::string>("1000ms");
    ins.timeout = Props::parse_interval<std::chrono::milliseconds>(val).count();

  } catch (const YAML::Exception &e) {
    LOG_ERROR("Error in Establishing INS Parameters");
    std::exit(0);
  }
}

void InitParameters::initialize_lidar(YAML::Node &config,
                                      SensorParams &params) {
  LOG_INFO("Establishing Lidar Parameters.");

  try {
    auto &lidar = params.lidar;
    lidar.in_use = true;

    lidar.ips = Props::parse_numbers<std::string>(config["ips"]);
    lidar.ports = Props::parse_numbers<std::string>(config["ports"]);
    lidar.expected_guage = config["expected_guage"].as<double>(1.452);
    lidar.expected_height_deviation =
        config["expected_height_deviation"].as<double>(0.0);

  } catch (const YAML::Exception &e) {
    LOG_ERROR("Error in Establishing Lidar Parameters");
    std::exit(0);
  }
}

void InitParameters::initialize_gpr(YAML::Node &config, SensorParams &params) {
  LOG_INFO("Establishing GPR Parameters.");

  try {
    auto &gpr = params.gpr;
    gpr.in_use = true;

    gpr.ips = Props::parse_numbers<std::string>(config["ips"]);
    gpr.ports = Props::parse_numbers<std::string>(config["ports"]);
    gpr.point_per_trace = config["point_per_trace"].as<int>(250);
    gpr.point_stacks = config["point_stacks"].as<int>(4);

    std::string val = config["time_sampling_interval"].as<std::string>("200ms");
    gpr.time_sampling_interval =
        Props::parse_interval<std::chrono::milliseconds>(val).count();

    val = config["period_s"].as<std::string>("0.00294s");
    gpr.period_s = Props::parse_interval<std::chrono::seconds>(val).count();

    gpr.first_break_point = config["first_break_point"].as<int>(0);
    gpr.output_batch = config["output_batch"].as<int>(10);
  } catch (const YAML::Exception &e) {
    LOG_ERROR("Error in Establishing Lidar Parameters");
    std::exit(0);
  }
}

void InitParameters::init_params(SensorParams &params,
                                 const std::string &launch_path) {
  YAML::Node config;
  try {
    // Load the YAML configuration file
    config = YAML::LoadFile(launch_path);

    std::filesystem::path file_path = std::filesystem::current_path();
    std::string rel_path = "config/dimensions.yaml";
    file_path /= rel_path;
    YAML::Node node_dims = YAML::LoadFile(file_path.string());

    if (node_dims["sensors"]) {
      for (const auto &ste : node_dims["sensors"]) {
        std::string sensor_type = ste.first.as<std::string>();
        for (const auto &sps : ste.second) {
          auto info = SensorTransform::load_frm_yaml(sps.second);

          transform_info[sensor_type][info.name] = std::move(info);
        }
      }
    }

  } catch (const YAML::Exception &e) {
    std::cerr << "Error loading YAML file (" << launch_path << "): " << e.what()
              << "\n";
  }

  InitParameters::initialize_general(config, params);

  LOG_INFO("Initializing Sensors.");

  if (config["sensors"]) {
    for (auto it = config["sensors"].begin(); it != config["sensors"].end();
         ++it) {

      std::string sensor_name = it->first.as<std::string>();
      if (sensor_init.find(sensor_name) != sensor_init.end()) {
        sensor_init[sensor_name](it->second, params);
      }
    }
  }

  LOG_INFO("Printing Sensor Information.");
  auto base = std::filesystem::path(params.general_storage);
  base /= ("sensor");

  try {
    std::filesystem::create_directories(base);
  } catch (const std::filesystem::filesystem_error &e) {
    std::ostringstream info_error;
    info_error << "Failed to create folder: " << base
               << "\nReason: " << e.what() << std::endl;
    LOG_ERROR(info_error.str());
  }

  for (const auto &[sensor_type, sensor_map] : transform_info) {
    std::string path_info = sensor_type + ".txt";
    std::filesystem::path out_path = base / path_info;
    SensorTransform::write_info(sensor_map, out_path.string());
  }

  LOG_INFO("End of Sensor Establishment. Application Begins");
}

// --------- RUN ALGORITHMS ALTERNATIVE --------- //
void InitParameters::initialize_log_params_alt(const std::string &path) {

  std::filesystem::path file_path = path;
  std::string msg = "atis_log.log";

  std::filesystem::path atis_log_path = file_path / msg;
  Logger::initialize(atis_log_path.string());

  msg = "spot_check_log.csv";
  std::filesystem::path scl = file_path / msg;
  params.spot_check_log = scl.string();

  msg = "acqusition_track.txt";
  scl = file_path / msg;
  params.acq_track_log = scl.string();

  // storing data storage
  params.data_storage = path;
  params.log_folder = path;
  params.general_storage = path;

  params.create_folders = false;
}
