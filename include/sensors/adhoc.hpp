#ifndef SENSORS_ADHOC_HPP
#define SENSORS_ADHOC_HPP

#include "controller.hpp"
#include "param.hpp"
#include "sensors/pools/unified_pool.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <tbb/concurrent_priority_queue.h>
#include <thread>
#include <unordered_set>
#include <vector>

namespace BaseClass {
// ---------- Base class for all sensors ------------ //
class Sensor {
  const Props::Sensors type;
  const std::string type_name;
  const std::string message_header;
  const int id;

  FlowControl::Type current_stage = FlowControl::Type::NONE;

public:
  Sensor(Props::Sensors type, const std::string &sname,
         FlowControl::Ptr ctrl = nullptr, int id = -1);

  virtual ~Sensor() = default;

  // initializing sensors
  virtual void initialize() = 0;

  // connection to sensors e.t.c
  virtual void get_packet() = 0;

  virtual void start() = 0;

  virtual void pause() = 0;

  virtual void stop() = 0;

  int get_id() const;

  Props::Sensors get_type() const;

  std::string get_sensor_name() const;

  void success_log(const std::string &msg);

  void error_log(const std::string &msg);

  void warn_log(const std::string &msg);

  bool break_condition();

  void enqueue_data(double timestamp);

  template <typename DataPtr>
  void add_to_task(std::unordered_map<double, std::vector<DataPtr>> &storage,
                   DataPtr &data, double t_data, double comp_time) {
    if (replace_tag) {
      current_tag =
          Props::bucket_time_start(t_data, comp_time, Props::TimeScale::SECS);
      replace_tag = false;
    }

    {
      std::unique_lock<std::shared_mutex> lock(data_mtx);
      storage[std::floor(current_tag)].emplace_back(data);
    }

    if (Props::time_diff(t_data, current_tag, Props::TimeScale::SECS) >=
        comp_time) {
      enqueue_data(std::floor(current_tag));
      replace_tag = true;
    }
  }

  template <typename DataPtr>
  std::optional<std::vector<DataPtr>>
  extract_data_at(std::unordered_map<double, std::vector<DataPtr>> &storage,
                  double timestamp) {
    std::unique_lock<std::shared_mutex> lock(data_mtx);
    auto it = storage.find(timestamp);
    if (it == storage.end())
      return std::nullopt;

    auto node = storage.extract(it);
    return std::move(node.mapped());
  }

  template <typename Container> void flush_all(Container &storage) {
    std::unique_lock<std::shared_mutex> lock(data_mtx);
    for (auto it = storage.begin(); it != storage.end(); ++it)
      enqueue_data(it->first);
  }

public:
  const std::string sname;

protected:
  void terminate_thread();

protected:
  FlowControl::Ptr ctrl;

  std::thread work_thread;

  TaskPool::Ptr task_pool = nullptr;

  bool is_stopped = false, replace_tag = true;

  double current_tag = 0;

  std::shared_mutex data_mtx;
};

// -------------- Base Manager ------------ //
class Manager {

  const std::string base_name;
  TaskFuncs::Set data_tracker;

  int num_sensors;

public:
  Manager(Props::Sensors type);

  virtual void initialize(FlowControl::Ptr ctrl = nullptr,
                          TaskPool::Ptr tp = nullptr) = 0;

  virtual void process_data(const TaskFuncs::Set &synced) = 0;

  std::optional<TaskFuncs::Set> update_data_count(int id, double timestamp);

  void success_log(const std::string &msg);

  void error_log(const std::string &msg);

  void warn_log(const std::string &msg);

  void set_num_count(int _num_sensors) { num_sensors = _num_sensors; }

  void create_directory(std::filesystem::path &dir);

  void create_file_helper(std::filesystem::path &file_path,
                          const std::string &header);

protected:
  std::mutex data_mutex;

  bool already_registered = false;
};

}; // namespace BaseClass

#endif
