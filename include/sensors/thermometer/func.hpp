#ifndef THERMOMETER_ATIS_DATA_HPP
#define THERMOMETER_ATIS_DATA_HPP

#include "sensors/adhoc.hpp"
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <tbb/concurrent_queue.h>
#include <termios.h>

struct ThermometerData {
  double time_data;
  double temperature;
  std::string time_stamp;
  using Ptr = std::shared_ptr<ThermometerData>;
};

namespace DataDevice {
class Thermometer : public BaseClass::Sensor {
  int baud_rate = -1, therm_add = 1;
  bool acquire_started = false;
  int port_number;
  std::string Tx;

public:
  using Ptr = std::unique_ptr<Thermometer>;

  Thermometer() = default;

  Thermometer(FlowControl::Ptr ctrl, int id, int port_id,
              TaskPool::Ptr tp = nullptr);

  ~Thermometer();

  // initializing sensors
  void initialize() override;

  // connection to sensors e.t.c
  void get_packet() override;

  void start() override;

  void pause() override;

  void stop() override;

  void run();

  std::optional<std::vector<ThermometerData::Ptr>> get_data(double timestamp);

private:
  std::string create_name(int port_id);

  double get_temperature();

private:
  std::unordered_map<double, std::vector<ThermometerData::Ptr>> storage;

  void open_port();

  void setup_transfer_info();

  void warm_up_stage();
};
} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
class ThermometerSystemManager
    : public BaseClass::Manager,
      public std::enable_shared_from_this<ThermometerSystemManager> {
  std::vector<DataDevice::Thermometer::Ptr> sensors;

  std::filesystem::path base_path;
  std::ofstream data_stream;
  std::mutex write_mtx;
  std::shared_mutex new_mtx;

  double last_batch_timestamp = 0.0;

public:
  using Ptr = std::shared_ptr<ThermometerSystemManager>;

  ThermometerSystemManager(FlowControl::Ptr ctrl = nullptr,
                           TaskPool::Ptr tp = nullptr);

  ~ThermometerSystemManager();

  void initialize(FlowControl::Ptr ctrl = nullptr,
                  TaskPool::Ptr tp = nullptr) override;

  void process_data(const TaskFuncs::Set &synced) override;

  static Ptr create(FlowControl::Ptr ctrl, TaskPool::Ptr tp);

  std::string create_header();

private:
  Ptr get_shared_data();
};

#endif
