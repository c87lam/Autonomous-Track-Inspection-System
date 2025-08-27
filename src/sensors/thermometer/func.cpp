#include "sensors/thermometer/func.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring> // For memset
#include <ctime>
#include <fcntl.h> // File control definitions
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <unistd.h>
#include <unordered_map>

#include <linux/serial.h>
#include <sys/ioctl.h>

namespace ThermoFuncs {
int get_baud_rate(const std::string &baud_rate_str) {
  // Map string values to termios baud rate constants
  static const std::unordered_map<std::string, int> baud_rate_map = {
      {"B9600", B9600},
      {"B19200", B19200},
      {"B38400", B38400},
      {"B57600", B57600},
      {"B115200", B115200}};

  auto it = baud_rate_map.find(baud_rate_str);
  if (it != baud_rate_map.end()) {
    return it->second;
  } else {
    std::ostringstream out;
    out << "Invalid baud rate: " << baud_rate_str << std::endl;
    out << "Allowed rates are: [B9600, B19200, B38400, B57600, B115200]"
        << std::endl;
    throw std::invalid_argument(out.str());
  }
}

} // namespace ThermoFuncs

namespace DataDevice {
Thermometer::Thermometer(FlowControl::Ptr ctrl, int id, int port_id,
                         TaskPool::Ptr tp)
    : BaseClass::Sensor(Props::Sensors::TEMP, create_name(port_id), ctrl, id) {

  baud_rate = ThermoFuncs::get_baud_rate(params.temp.baud_rate);
  initialize();
  task_pool = tp;
}

std::string Thermometer::create_name(int port_id) {
  try {
    if (port_id <= 0)
      throw std::invalid_argument("Port Id should be from [1 .. N]");
    // zeroed out port id
    return "/dev/ttyS" + std::to_string(port_id - 1);
  } catch (const std::exception &e) {
    error_log("Exception in create_name: " + std::string(e.what()));
    std::exit(EXIT_FAILURE);
  }
}

Thermometer::~Thermometer() {
  stop();
  flush_all(storage);
  terminate_thread();
}

double Thermometer::get_temperature() {
  double final_temp = std::numeric_limits<double>::max();
  try {

    if (static_cast<int>(tcdrain(port_number)) != 0) {
      error_log("Cant drain the port");
      return final_temp;
    }

    if (static_cast<int>(write(port_number, Tx.c_str(), Tx.size())) < 0) {
      error_log("Error writing to port: " + std::string(strerror(errno)));
      return final_temp;
    }

    char response[256];
    int size = read(port_number, response, sizeof(response));

    if (size == 2) {
      uint16_t combined = (static_cast<unsigned char>(response[0]) << 8) |
                          static_cast<unsigned char>(response[1]);
      final_temp = (combined - 1000) / 10.0;
    } else {
      error_log("Unexpected data length recieved");
    }
  } catch (const std::exception &e) {
    error_log("Error when getting temperature: " + std::string(e.what()));
  }

  return final_temp;
}

void Thermometer::warm_up_stage() {
  int idx = 0;
  while (idx++ < 5) {
    auto temp_val = get_temperature();
    (void)temp_val;

    // wait to wakeup
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Thermometer::open_port() {
  // open the port, avoid it from recieinv signals, allow for synchronous
  // writes
  port_number = open(get_sensor_name().c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (port_number < 0)
    throw std::runtime_error("Error opening port.");

  // get and set data bit information
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(port_number, &tty) != 0) {
    throw std::runtime_error("error from tcgetattr" +
                             std::string(strerror(errno)));
  }

  // Set baud rate for input and output and ensure baud rate setting is applied
  cfsetospeed(&tty, baud_rate);
  cfsetispeed(&tty, baud_rate);

  // Configure data bits (8-bit characters)
  tty.c_cflag &= ~CSIZE; // Clear current character size mask
  tty.c_cflag |= CS8;    // Set 8-bit characters

  // Disable parity checking
  tty.c_cflag &= ~(PARENB | PARODD);

  // Set one stop bit
  tty.c_cflag &= ~CSTOPB;

  // Disable hardware flow control (RTS/CTS)
  tty.c_cflag &= ~CRTSCTS;

  // Enable receiver and set local mode (ignores modem status lines)
  tty.c_cflag |= (CLOCAL | CREAD);

  // Disable software flow control (XON/XOFF)
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Disable break processing
  tty.c_iflag &= ~IGNBRK;

  // Set raw input/output mode (disable canonical mode and echo)
  tty.c_lflag = 0; // No signal handling, no echo, no canonical mode
  tty.c_oflag = 0; // No special output processing

  // Configure read timeout settings
  tty.c_cc[VMIN] = 0;  // Minimum number of characters to read
  tty.c_cc[VTIME] = 5; // Timeout in tenths of a second (0.5s)

  if (tcsetattr(port_number, TCSANOW, &tty) != 0) {
    throw std::runtime_error("error from tcsetattr" +
                             std::string(strerror(errno)));
  }
}

void Thermometer::setup_transfer_info() {
  int tmp = (therm_add & 0xf) + 0xb0;
  tmp = ((tmp << 8) & 0xff00) + 0x01;
  Tx = std::string(reinterpret_cast<char *>(&tmp), 2);
}

// initializing sensors
void Thermometer::initialize() {
  // Step 1: Open serial port
  try {
    open_port();

    setup_transfer_info();

    warm_up_stage();

    success_log("Configured Successfully. Initial temperature: " +
                std::to_string(get_temperature()));
    work_thread = std::thread(&Thermometer::run, this);

  } catch (const std::exception &e) {
    error_log("Exception in Thermometer::initialize(): " +
              std::string(e.what()));
    stop();
    std::exit(EXIT_FAILURE);
  } catch (...) {
    error_log("Unknown exception occurred during Thermometer::initialize()");
    stop();
    std::exit(EXIT_FAILURE);
  }
}

// connection to sensors e.t.c
void Thermometer::get_packet() {

  auto [time_double, string_time] = Props::create_timestamp();
  double c_temp = get_temperature();
  if (c_temp == std::numeric_limits<double>::max())
    return;

  auto res = std::make_shared<ThermometerData>();
  {
    res->time_data = time_double;
    res->temperature = c_temp;
    res->time_stamp = string_time;
  }

  // add to task queue
  add_to_task(storage, res, time_double, params.data_batch_time);
}

void Thermometer::start() {
  if (acquire_started)
    return;

  acquire_started = true;
  success_log("Data Acquisition Started.");
}

void Thermometer::pause() {
  if (!acquire_started)
    return;

  acquire_started = false;
  success_log("Data Acquisition Paused.");
}

void Thermometer::stop() {
  if (port_number >= 0) {
    tcdrain(port_number);
    tcflush(port_number, TCIFLUSH);
    close(port_number);
  }

  success_log("Data Acquisition Stopped.");
}

std::optional<std::vector<ThermometerData::Ptr>>
Thermometer::get_data(double timestamp) {
  return extract_data_at<ThermometerData::Ptr>(storage, timestamp);
}

void Thermometer::run() {
  success_log("Start of Dedicated Thread.");
  auto next_time = std::chrono::steady_clock::now();

  while (!ctrl->get_state(FlowControl::Type::STOP)) {
    {
      std::unique_lock<std::mutex> lock(ctrl->mtx);
      ctrl->cv.wait(lock, [&] {
        return ctrl->get_state(FlowControl::Type::RUN) || break_condition();
      });
    }

    if (ctrl->get_state(FlowControl::Type::STOP)) {
      pause();
      break;
    }

    if (ctrl->get_state(FlowControl::Type::RUN)) {
      start();

      try {
        get_packet();
      } catch (const std::exception &e) {
        error_log("Error in run(): " + std::string(e.what()));
      }

      next_time += params.temp.sampling_period;
      std::this_thread::sleep_until(next_time);
    }

    if (ctrl->get_state(FlowControl::Type::PAUSE))
      pause();
  }

  stop(); // stopping system
}
} // namespace DataDevice
  // ----------------------------------- //
  // -------------- Manager ------------ //
  // ----------------------------------- //

ThermometerSystemManager::ThermometerSystemManager(FlowControl::Ptr ctrl,
                                                   TaskPool::Ptr tp)
    : BaseClass::Manager(Props::Sensors::TEMP) {
  std::filesystem::path file_path = params.data_storage;
  file_path /= "Temperature";
  base_path = file_path;

  Props::ensure_dir(base_path.string());
  initialize(ctrl, tp);
}

ThermometerSystemManager::~ThermometerSystemManager() {
  success_log("Ending Thermometer System.");
}

std::string ThermometerSystemManager::create_header() {
  std::ostringstream header;

  header << "### Sensor Index Mapping ###\n";
  for (size_t i = 0; i < sensors.size(); ++i) {
    header << std::setw(3) << "Thermometer " << i + 1 << " -> "
           << sensors[i]->get_sensor_name() << "\n";
  }
  header << "################################\n\n";

  header << "Timestamp (" << Props::TIME_FMT << ")";
  for (size_t i = 1; i <= sensors.size(); ++i) {
    header << ", Thermometer" << i << " (°C)";
  }

  header << "\n";

  return header.str();
}

ThermometerSystemManager::Ptr ThermometerSystemManager::get_shared_data() {
  return shared_from_this();
}

void ThermometerSystemManager::initialize(FlowControl::Ptr ctrl,
                                          TaskPool::Ptr tp) {
  if (!sensors.empty()) {
    warn_log("Already discovered");
    return;
  }

  // .... Initializing individual systems .....
  sensors.resize(params.temp.port_numbers.size());
  for (size_t idx = 0; idx < params.temp.port_numbers.size(); ++idx) {
    sensors[idx] = std::make_unique<DataDevice::Thermometer>(
        ctrl, idx, params.temp.port_numbers[idx], tp);
  }

  // one csv file
  set_num_count(sensors.size());
  success_log("Initialization Complete.");
}

void ThermometerSystemManager::process_data(const TaskFuncs::Set &synced) {

  std::unordered_map<int, std::vector<ThermometerData::Ptr>> vals;
  size_t valid_num = 0;
  size_t min_count = std::numeric_limits<size_t>::max();
  double avg_ts = 0;

  for (const auto &temp_info : synced) {
    auto data_opt = sensors[temp_info.id]->get_data(temp_info.timestamp);
    if (!data_opt || data_opt->empty())
      continue;

    ++valid_num;
    avg_ts += temp_info.timestamp;

    min_count = std::min(min_count, data_opt->size());
    vals[temp_info.id] = std::move(*data_opt);
  }

  if (valid_num != sensors.size()) {
    std::ostringstream warning;
    warning << "Partial sensor sync. Expected" << sensors.size() << " got "
            << valid_num << std::endl;
    warn_log(warning.str());
    return;
  }

  avg_ts /= static_cast<double>(valid_num);
  if (avg_ts < params.init_complete_time)
    return;

  std::filesystem::path store_folder;
  {
    double folder_ts = std::floor(avg_ts);
    store_folder = base_path / (Props::format_timestamp(folder_ts) + ".csv");
  }

  // Write header only if file does not exist
  bool needs_header = !std::filesystem::exists(store_folder.string());
  std::ofstream output(store_folder.string(), std::ios::app);
  if (needs_header)
    output << create_header();

  for (size_t idx = 0; idx < min_count; ++idx) {
    double data_ts_avg = 0;
    std::ostringstream line_data;
    for (size_t s_idx = 0; s_idx < sensors.size(); ++s_idx) {
      data_ts_avg += vals[s_idx][idx]->time_data;
      line_data << vals[s_idx][idx]->temperature << ", ";
    }

    data_ts_avg /= static_cast<double>(valid_num);
    if (data_ts_avg < params.init_complete_time)
      continue;
    output << data_ts_avg << ", " << line_data.str() << "\n";
  }
}

ThermometerSystemManager::Ptr
ThermometerSystemManager::create(FlowControl::Ptr ctrl, TaskPool::Ptr tp) {
  auto new_tm = std::make_shared<ThermometerSystemManager>(ctrl, tp);
  LOG_INFO("Registering task.");
  tp->register_manager(Props::Sensors::TEMP, new_tm);
  LOG_INFO("Thermometer Manger Task Registered.");

  return new_tm;
}
