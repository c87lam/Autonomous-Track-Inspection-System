#include "sensors/adhoc.hpp"

// ---------- Base class for all sensors ------------ //
namespace BaseClass {

Sensor::Sensor(Props::Sensors type, const std::string &n_name,
               FlowControl::Ptr _ctrl, int id)
    : type(type), type_name(Props::sensors_to_string(type)),
      message_header(type_name + "-" + n_name + ": "), id(id), sname(n_name),
      ctrl(_ctrl ? _ctrl : std::make_shared<FlowControl>()) {}

int Sensor::get_id() const { return id; };

Props::Sensors Sensor::get_type() const { return type; };

std::string Sensor::get_sensor_name() const { return sname; }

void Sensor::success_log(const std::string &msg) {
  LOG_INFO("{}{}", message_header, msg);
}

void Sensor::error_log(const std::string &msg) {
  LOG_ERROR("{}{}", message_header, msg);
}

void Sensor::warn_log(const std::string &msg) {
  LOG_WARN("{}{}", message_header, msg);
}

bool Sensor::break_condition() {
  auto curr_stage = ctrl->current_stage();
  if (current_stage == curr_stage)
    return false;

  current_stage = curr_stage;
  return true;
}

void Sensor::enqueue_data(double timestamp) {
  if (!task_pool) {
    warn_log("Taskpool not initialized.");
    return;
  }

  task_pool->enqueue(type, timestamp, id);
}

void Sensor::terminate_thread() {
  if (work_thread.joinable()) {
    work_thread.join();
  }

  success_log("Terminated.");
}

// -------------- Base Manager ------------ //
Manager::Manager(Props::Sensors type)
    : base_name(std::string(Props::sensors_to_string(type)) + " Manager: ") {}

void Manager::success_log(const std::string &msg) {
  LOG_INFO("{}{}", base_name, msg);
}

void Manager::error_log(const std::string &msg) {
  LOG_ERROR("{}{}", base_name, msg);
}

void Manager::warn_log(const std::string &msg) {
  LOG_WARN("{}{}", base_name, msg);
}

std::optional<TaskFuncs::Set> Manager::update_data_count(int id,
                                                         double timestamp) {
  std::unique_lock<std::mutex> lock(data_mutex);

  bool inserted = data_tracker.emplace(id, timestamp).second;

  if (!inserted || data_tracker.size() >= static_cast<size_t>(num_sensors)) {
    TaskFuncs::Set result = std::move(data_tracker);
    data_tracker.clear();

    if (!inserted)
      data_tracker.emplace(id, timestamp);

    return result;
  }

  return std::nullopt;
}

void Manager::create_directory(std::filesystem::path &dir) {
  if (std::filesystem::exists(dir))
    return;

  try {
    std::filesystem::create_directories(dir);
  } catch (const std::filesystem::filesystem_error &e) {
    std::ostringstream msg;
    msg << "Error: Failed to create directory '" << dir.string()
        << "' due to: " << e.what() << "\n";

    error_log(msg.str());
  }
}

void Manager::create_file_helper(std::filesystem::path &file_path,
                                 const std::string &header) {
  std::ofstream file(file_path.string());

  if (!file.is_open()) {
    std::string msg = "Failed to create CSV file: " + file_path.string();
    throw std::runtime_error(msg);
  }

  file << header;
  file.close();

  success_log("File successfull created at: " + file_path.string());
}
}; // namespace BaseClass
