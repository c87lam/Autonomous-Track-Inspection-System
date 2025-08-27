#ifndef INS_ATIS_DATA_HPP
#define INS_ATIS_DATA_HPP

#include "IMUDriver.h"
#include "helper.hpp"
#include "sensors/adhoc.hpp"
#include <array>
#include <deque>
#include <filesystem>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <tbb/concurrent_queue.h>

namespace DataDevice {
class INS : public BaseClass::Sensor {
  IMU::Driver driver;
  bool acquire_started = false;
  uint8_t packet_type = IMU::PacketType::IMU_Orientation;

  double time_double = 0.0;
  std::string string_stamp;
  Eigen::Matrix3d rot_vec = Eigen::Matrix3d::Identity();

  std::vector<ExtractedData> init_buffer;
  bool tracker_initialized = false;
  Curvature::ArcDevTracker::Ptr dev_track;

public:
  using Ptr = std::unique_ptr<INS>;

  INS() = default;

  INS(FlowControl::Ptr ctrl, const std::string &url, int id,
      std::string &folder, TaskPool::Ptr tp = nullptr);

  ~INS();

  // initializing sensors
  void initialize() override;

  // connection to sensors e.t.c
  void get_packet() override;

  void start() override;

  void pause() override;

  void stop() override;

  std::optional<std::vector<INSData::Ptr>> get_data(double timestamp);

  void run();

  static Eigen::Vector3d convert_rpy(const Eigen::Matrix3d &rot_vec,
                                     const double &roll, const double &pitch,
                                     const double &yaw);

  static double wrap180(double angle_deg);

  static double wrap360(double angle_deg);

  static Eigen::Vector3d rotm_to_euler(const Eigen::Matrix3d &rot);

  static std::string get_csv_header();

private: // function
  std::optional<ExtractedData> convert(const IMU::INSDataStruct *data);

  void automatic_data_control(double &curr_speed);

  static void callback(IMU::INSDataStruct *data, void *context);

  void handle_initialization(ExtractedData &&data);

  void handle_new_data(ExtractedData &&data);

private: // attribute
  std::unordered_map<double, std::vector<INSData::Ptr>> storage;
};
} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
class InertialSystemManager
    : public BaseClass::Manager,
      public std::enable_shared_from_this<InertialSystemManager> {
  int ins_counter = 0;
  std::filesystem::path base_path;
  std::vector<DataDevice::INS::Ptr> sensors;
  std::vector<std::string> ins_names;

public:
  using Ptr = std::shared_ptr<InertialSystemManager>;

  InertialSystemManager(FlowControl::Ptr ctrl = nullptr,
                        TaskPool::Ptr tp = nullptr);

  ~InertialSystemManager();

  void initialize(FlowControl::Ptr ctrl = nullptr,
                  TaskPool::Ptr tp = nullptr) override;

  void process_data(const TaskFuncs::Set &synced) override;

  static Ptr create(FlowControl::Ptr ctrl, TaskPool::Ptr tp);
};
#endif
