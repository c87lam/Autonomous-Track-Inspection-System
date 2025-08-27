#include "sensors/ins/func.hpp"
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
inline std::string extract_device_name(const std::string &input) {
  // Find the position of "/dev/"
  std::size_t start_pos = input.find("/dev/");
  if (start_pos == std::string::npos) {
    throw std::invalid_argument("Invalid string format: /dev/ not found");
  }

  // Move past "/dev/" (length = 5)
  start_pos += 5;

  // Find the next ':' after "/dev/"
  std::size_t end_pos = input.find(':', start_pos);
  if (end_pos == std::string::npos) {
    throw std::invalid_argument("Invalid string format: No closing ':' found");
  }

  // Extract the device name
  return input.substr(start_pos, end_pos - start_pos);
}
} // namespace

namespace DataDevice {

INS::INS(FlowControl::Ptr ctrl, const std::string &url, int id,
         std::string &folder, TaskPool::Ptr tp)
    : BaseClass::Sensor(Props::Sensors::INS, url, ctrl, id) {
  initialize();
  task_pool = tp;

  rot_vec = transform_info["ins"][sname].transform.topLeftCorner<3, 3>();

  dev_track = std::make_shared<Curvature::ArcDevTracker>(
      std::vector<double>{9.4488, 18.8976},
      transform_info["ins"][sname].transform, folder);

  // essentially the running thread for the algorithm
  driver.setCallback(&INS::callback, this);
  pause();
}

INS::~INS() {
  stop();
  flush_all(storage);
  terminate_thread();
}

// initializing sensors
void INS::initialize() {
  if (driver.connect(get_sensor_name().c_str()) != 0) {
    error_log("Failed Connection");
    std::exit(EXIT_FAILURE);
  }

  success_log("Initialization Complete.");

  work_thread = std::thread(&INS::run, this);
}

double INS::wrap180(double angle_deg) {
  while (angle_deg > 180.0)
    angle_deg -= 360.0;
  while (angle_deg < -180.0)
    angle_deg += 360.0;
  return angle_deg;
}

double INS::wrap360(double angle_deg) {
  while (angle_deg < 0.0)
    angle_deg += 360.0;
  while (angle_deg >= 360.0)
    angle_deg -= 360.0;
  return angle_deg;
}

Eigen::Vector3d INS::rotm_to_euler(const Eigen::Matrix3d &rot) {
  double sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  double x, y, z;

  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  } else {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }

  Eigen::Vector3d ang(x, y, z);
  return ang;
}

Eigen::Vector3d INS::convert_rpy(const Eigen::Matrix3d &rot_vec,
                                 const double &roll, const double &pitch,
                                 const double &yaw) {
  // ----------------- //
  Eigen::Quaterniond q_enu =
      Eigen::Quaterniond(Props::deg_rpy2rot(roll, pitch, yaw));

  Eigen::Quaterniond q_frame(rot_vec);
  Eigen::Quaterniond q_rhr = q_frame * q_enu;

  Eigen::Matrix3d R_rhr = q_rhr.toRotationMatrix();
  Eigen::Vector3d rpy = INS::rotm_to_euler(R_rhr);
  Eigen::Vector3d rpy_deg = rpy * (180.0 / M_PI);

  rpy_deg[0] = INS::wrap180(rpy_deg[0]);
  rpy_deg[1] = INS::wrap180(rpy_deg[1]);
  rpy_deg[2] = INS::wrap360(rpy_deg[2]);

  return rpy_deg;
}

// connection to sensors e.t.c
std::optional<ExtractedData> INS::convert(const IMU::INSDataStruct *data) {
  if (!data)
    return std::nullopt;

  ExtractedData out;
  // convert to right- hand coordinates
  Eigen::Vector3d rpy =
      INS::convert_rpy(rot_vec, data->Roll, data->Pitch, data->Heading);

  // Apply rotation to the vectors
  Eigen::Vector3d gyro(data->Gyro[0], data->Gyro[1], data->Gyro[2]);
  Eigen::Vector3d acc(data->Acc[0], data->Acc[1], data->Acc[2]);
  Eigen::Vector3d mag(data->Mag[0], data->Mag[1], data->Mag[2]);

  gyro = rot_vec * gyro;
  acc = rot_vec * acc;
  mag = rot_vec * mag;

  // data transfer
  out.rpy = {rpy.x(), rpy.y(), rpy.z()};
  out.lat_lon_alt = {data->LatGNSS, data->LonGNSS, data->AltGNSS};
  out.gyro = {gyro.x(), gyro.y(), gyro.z()};
  out.acc = {acc.x(), acc.y(), acc.z()};
  out.mag = {mag.x(), mag.y(), mag.z()};
  out.speed = {data->V_Hor, data->V_ver};
  out.overall_speed = std::hypot(data->V_Hor, data->V_ver);

  // Allows tracking for spot checking
  ctrl->add_data(data->LatGNSS, data->LonGNSS, data->AltGNSS);
  return out;
}

void INS::start() {
  if (acquire_started)
    return;

  acquire_started = true;
  success_log("Data Acquisition Started.");

  driver.start(packet_type, true);
}

void INS::pause() {
  if (!acquire_started)
    return;

  acquire_started = false;
  success_log("Data Acquisition Paused.");
}

void INS::stop() {
  if (is_stopped)
    return;

  is_stopped = true;
  driver.disconnect();
  success_log("Data Acquisition Stopped");
}

void INS::automatic_data_control(double &curr_speed) {
  if (ctrl->forced_action_check())
    return; // we are in manual control mode.

  // should check if we can based on current speed and threshold
  bool can_record = curr_speed > params.min_speed_for_acq;
  bool currently_running = ctrl->current_stage() == FlowControl::Type::RUN;

  if (currently_running == can_record)
    return;

  if (can_record) {
    ctrl->run();
    success_log("State changed to start recording.");
  } else {
    ctrl->pause();
    success_log("State changed to stop recording.");
  }
}

void INS::handle_initialization(ExtractedData &&data) {
  init_buffer.push_back(std::move(data));

  if (init_buffer.size() < 10)
    return;

  dev_track->init_origin(init_buffer);

  tracker_initialized = true;
  success_log("Deviation trackers initialized.");

  init_buffer.clear();
  storage.clear();
}

void INS::handle_new_data(ExtractedData &&data) {

  automatic_data_control(data.overall_speed);

  dev_track->update(data);

  auto res = std::make_shared<INSData>();
  {
    res->name = sname;
    res->time = time_double;
    res->timestamp = string_stamp;
    res->ins_data = data.csv_string();
    res->id = get_id();
  }

  // add to task queue
  add_to_task(storage, res, time_double, params.data_batch_time);
}

void INS::callback(IMU::INSDataStruct *data, void *context) {
  auto *instance = static_cast<INS *>(context);
  if (!data)
    return;

  auto opt_res = instance->convert(data);
  if (!opt_res)
    return;

  ExtractedData new_data = *opt_res;
  new_data.time = instance->time_double;
  new_data.timestamp = instance->string_stamp;

  if (!instance->tracker_initialized)
    instance->handle_initialization(std::move(new_data));
  else
    instance->handle_new_data(std::move(new_data));
}

void INS::get_packet() {
  std::tie(time_double, string_stamp) = Props::create_timestamp();
  driver.request(packet_type, params.ins.timeout);
}

void INS::run() {
  const auto loop_period =
      std::chrono::milliseconds(1000 / params.ins.frequency);

  auto next_loop_time = std::chrono::steady_clock::now() + loop_period;
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
      get_packet();
    }

    if (ctrl->get_state(FlowControl::Type::PAUSE))
      pause();

    std::this_thread::sleep_until(next_loop_time);
    next_loop_time += loop_period;
  }
}

std::optional<std::vector<INSData::Ptr>> INS::get_data(double timestamp) {
  return extract_data_at(storage, timestamp);
}

std::string INS::get_csv_header() {
  std::ostringstream header;
  header << "Timestamp (" << Props::TIME_FMT << "),"
         << "Horizontal Speed (m/s),"
         << "Vertical Speed (m/s),"
         << "Roll (deg),"
         << "Pitch (deg),"
         << "Yaw (deg),"
         << "Latitude,"
         << "Longitude,"
         << "Altitude (m),"
         << "Gyroscope X (rad/s),"
         << "Gyroscope Y (rad/s),"
         << "Gyroscope Z (rad/s),"
         << "Accelerometer X (m/s²),"
         << "Accelerometer Y (m/s²),"
         << "Accelerometer Z (m/s²),"
         << "Magnetometer X (µT),"
         << "Magnetometer Y (µT),"
         << "Magnetometer Z (µT)"
         << "\n";

  return header.str();
}

}; // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
InertialSystemManager::InertialSystemManager(FlowControl::Ptr ctrl,
                                             TaskPool::Ptr tp)
    : BaseClass::Manager(Props::Sensors::INS) {
  base_path = params.data_storage + "/INS";
  initialize(ctrl, tp);
}

InertialSystemManager::~InertialSystemManager() {}

void InertialSystemManager::initialize(FlowControl::Ptr ctrl,
                                       TaskPool::Ptr tp) {

  if (!sensors.empty()) {
    warn_log("Already discovered");
    return;
  }

  ins_names.resize(params.ins.urls.size());

  for (size_t idx = 0; idx < params.ins.urls.size(); ++idx) {
    try {
      std::filesystem::path new_path =
          base_path / ("INS_" + std::to_string(idx));
      ins_names[idx] = new_path.string();

      sensors.emplace_back(std::make_unique<DataDevice::INS>(
          ctrl, params.ins.urls[idx], ins_counter++, ins_names[idx], tp));

    } catch (const std::exception &e) {
      std::string msg =
          "Error Connecting to" + params.ins.urls[idx] + " : " + e.what();
      error_log(msg);
      std::exit(EXIT_FAILURE);
    }
  }

  set_num_count(sensors.size());
  success_log("Inititalization Complete!");
}

void InertialSystemManager::process_data(const TaskFuncs::Set &synced) {
  for (const auto &ins_info : synced) {
    auto data_opt = sensors[ins_info.id]->get_data(ins_info.timestamp);
    if (!data_opt || data_opt->empty())
      continue;

    const auto &data_vec = *data_opt;

    std::string ins_loc;
    {
      std::filesystem::path store_folder;
      double folder_ts = std::floor(ins_info.timestamp); // fixed this
      store_folder =
          ins_names[ins_info.id] + "/" + Props::format_timestamp(folder_ts);
      // ------------------------------------- //
      ins_loc = store_folder.string() + ".csv";
    }

    // Write header only if file does not exist
    bool needs_header = !std::filesystem::exists(ins_loc);
    std::ofstream output(ins_loc, std::ios::app);
    if (needs_header)
      output << DataDevice::INS::get_csv_header();

    for (const auto &data : data_vec) {
      if (data->time < params.init_complete_time)
        continue;

      output << data->ins_data;
    }
  }
}

InertialSystemManager::Ptr InertialSystemManager::create(FlowControl::Ptr ctrl,
                                                         TaskPool::Ptr tp) {
  auto new_im = std::make_shared<InertialSystemManager>(ctrl, tp);
  LOG_INFO("Registering task.");
  tp->register_manager(Props::Sensors::INS, new_im);
  LOG_INFO("INS Manger Task Registered.");

  return new_im;
}
