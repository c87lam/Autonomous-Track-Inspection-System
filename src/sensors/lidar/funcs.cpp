#include "sensors/lidar/func.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tbb/parallel_for_each.h>
#include <utility>
#include <vector>

namespace {
constexpr int BUFFER_SIZE = 2048; // buffer size based on scanner size

// -------- For printing csv -------- //
template <typename... Args> std::string join_csv(const Args &...args) {
  std::ostringstream oss;
  oss.precision(6);
  oss << std::fixed;

  ((oss << args << ","), ...);
  auto result = oss.str();
  result.pop_back(); // remove trailing comma
  return result;
}
} // namespace

namespace DataDevice {

Lidar::Lidar(FlowControl::Ptr ctrl, const std::string &ip,
             const std::string &port, int id, TaskPool::Ptr tp)
    : BaseClass::Sensor(Props::Sensors::LIDAR, create_name(ip, port), ctrl, id),
      ip(ip), port(port) {
  initialize();
  task_pool = tp;

  transform_mat =
      SensorTransform::invert_transform(transform_info["lidar"][ip].transform);
  cntr_dir = transform_info["lidar"][ip].sensor_location;
}

Lidar::~Lidar() {
  stop();
  flush_all(storage);
  terminate_thread();
}

std::string Lidar::create_name(const std::string &ip, const std::string &port) {
  return ip + ":" + port;
}

bool Lidar::connection_status() {
  if (!scanner_handle)
    return false;

  int status = 0;
  Lidar_GetConnectStatus(scanner_handle, &status);

  return (status == 3);
}

void Lidar::initialize() {
  try {
    scanner_handle =
        Lidar_Connect((char *)ip.c_str(), (char *)port.c_str(), 0);

    if (!scanner_handle) {
      throw std::runtime_error("Failed to get handle connection.");
    }

    auto start_time = std::chrono::system_clock::now();
    auto timeout = std::chrono::seconds(3);
    bool connected = false;
    while (std::chrono::system_clock::now() - start_time < timeout) {
      if (connection_status()) {
        connected = true;

        success_log("Sensor Connected.");
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!connected)
      throw std::runtime_error("Connected but no response.");

  } catch (const std::exception &e) {
    error_log("Exception during connection: " + std::string(e.what()));
    std::exit(EXIT_FAILURE);

  } catch (...) {
    error_log("Unknown error occurred while connecting.");
    std::exit(EXIT_FAILURE);
  }

  success_log("Initialization Complete.");

  work_thread = std::thread(&Lidar::run, this);
}

void Lidar::get_packet() {
  auto [time_double, string_stamp] = Props::create_timestamp();
  auto res = std::make_shared<LidarData>();

  // Options for reading
  unsigned int encoder = 0;
  unsigned char usr_io = 0;
  unsigned char raw_buffer[BUFFER_SIZE] = {0};
  int image_count = 0;

  std::vector<double> x_data(BUFFER_SIZE);
  std::vector<double> z_data(BUFFER_SIZE);
  std::vector<int> intensity(BUFFER_SIZE);
  std::vector<int> peak_width(BUFFER_SIZE);

  // Note - 1000 ms used as timeout
  int result = Lidar_GetXZIExtended(
      scanner_handle, x_data.data(), z_data.data(), intensity.data(),
      peak_width.data(), BUFFER_SIZE, &encoder, &usr_io, 1000, raw_buffer,
      BUFFER_SIZE, &image_count);

  if (result < 0) {
    error_log("Failed to get scan data from Lidar");
    return;
  }

  ProfilePrep profile;

  // Populate 3d coordinates and Prepare for image
  profile.conv_points = Eigen::MatrixXd::Zero(x_data.size(), 3);

  auto &pts_to_fill = profile.conv_points;

  res->orig_rows.reserve(BUFFER_SIZE);
  res->transformed_rows.reserve(BUFFER_SIZE);

  for (size_t idx = 0; idx < x_data.size(); ++idx) {
    double x = x_data[idx] / 1000.0;
    double z = z_data[idx] / 1000.0;

    Eigen::Vector4d raw_point(x, 0.0, z, 1.0);
    Eigen::Vector3d transformed = (transform_mat * raw_point).head<3>();

    pts_to_fill.row(idx) = transformed.transpose();

    profile.min_y = std::min(profile.min_y, transformed.y());
    profile.min_z = std::min(profile.min_z, transformed.z());
    profile.max_y = std::max(profile.max_y, transformed.y());
    profile.max_z = std::max(profile.max_z, transformed.z());

    std::string orig_row = join_csv(string_stamp, x_data[idx], z_data[idx],
                                    intensity[idx], peak_width[idx]);

    std::string trans_row =
        join_csv(string_stamp, transformed[0], transformed[1], transformed[2],
                 intensity[idx], peak_width[idx]);

    if (cntr_dir == Props::Location::LEFT) {
      orig_row += ",";
      trans_row += ",";
    }

    res->orig_rows.emplace_back(std::move(orig_row));
    res->transformed_rows.emplace_back(std::move(trans_row));
  }

  // extract the surfaces
  profile.cnt_dir = cntr_dir;
  res->contour = profile.surfaces(false);
  if (!res->contour.is_valid)
    return;

  // extract the surfaces
  res->time_data = time_double;
  res->timestamp = string_stamp;
  res->name = get_sensor_name();
  res->cnt_dir = cntr_dir;

  // add to task queue
  add_to_task(storage, res, time_double, params.data_batch_time);
}

void Lidar::start() {
  if (acquire_started)
    return;

  acquire_started = true;
  success_log("Data Acquisition Started.");
}

void Lidar::pause() {
  if (!acquire_started)
    return;

  acquire_started = false;
  success_log("Data Acquisition Paused.");
}

void Lidar::stop() {
  if (scanner_handle) {
    Lidar_Disconnect(scanner_handle);
    scanner_handle = nullptr;

    success_log("Data Acquisition Stopped.");
  }
}

std::optional<std::vector<LidarData::Ptr>> Lidar::get_data(double timestamp) {
  return extract_data_at(storage, timestamp);
}

void Lidar::run() {
  success_log("Start of Dedicated Thread.");

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

    else if (ctrl->get_state(FlowControl::Type::PAUSE))
      pause();
  }

  stop(); // stopping system
}

} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
LidarSystemManager::LidarSystemManager(FlowControl::Ptr ctrl, TaskPool::Ptr tp)
    : BaseClass::Manager(Props::Sensors::INS) {
  base_path = params.data_storage;
  base_path /= "Lidar";

  initialize(ctrl, tp);
}

LidarSystemManager::Ptr LidarSystemManager::get_shared_data() {
  return shared_from_this();
}

LidarSystemManager::~LidarSystemManager() {
  success_log("Ending Lidar System.");
}

void LidarSystemManager::initialize(FlowControl::Ptr ctrl, TaskPool::Ptr tp) {
  if (!sensors.empty()) {
    warn_log("Already discovered");
    return;
  }

  // .... Initializing individual systems .....
  sensors.resize(params.lidar.ips.size());
  for (size_t idx = 0; idx < params.lidar.ips.size(); ++idx) {
    sensors[idx] = std::make_unique<DataDevice::Lidar>(
        ctrl, params.lidar.ips[idx], params.lidar.ports[idx], idx, tp);
  }

  set_num_count(sensors.size());
  success_log("Initialization Complete.");
}

LidarSystemManager::Ptr LidarSystemManager::create(FlowControl::Ptr ctrl,
                                                   TaskPool::Ptr tp) {
  auto new_lm = std::make_shared<LidarSystemManager>(ctrl, tp);
  LOG_INFO("Registering task.");

  tp->register_manager(Props::Sensors::LIDAR, new_lm);
  LOG_INFO("Lidar Manger Task Registered.");

  return new_lm;
}

void LidarSystemManager::write_original(
    std::ostringstream &buffer,
    const std::map<Props::Location, DataToPrint> &data) {

  try {
    const auto &left_rows = data.at(Props::Location::LEFT).orig_rows;
    const auto &right_rows = data.at(Props::Location::RIGHT).orig_rows;

    size_t min_rows = std::min(left_rows.size(), right_rows.size());

    if (left_rows.size() != right_rows.size()) {
      LOG_WARN("LEFT/RIGHT orig_rows size mismatch: L={}, R={}",
               left_rows.size(), right_rows.size());
    }

    for (size_t i = 0; i < min_rows; ++i)
      buffer << left_rows[i] << right_rows[i] << "\n";

  } catch (const std::out_of_range &e) {
    LOG_WARN("Missing LEFT or RIGHT in write_original: {}", e.what());
  }
}

void LidarSystemManager::write_transformed(
    std::ostringstream &buffer,
    const std::map<Props::Location, DataToPrint> &data) {

  try {
    const auto &left_rows = data.at(Props::Location::LEFT).transformed_rows;
    const auto &right_rows = data.at(Props::Location::RIGHT).transformed_rows;

    size_t min_rows = std::min(left_rows.size(), right_rows.size());

    if (left_rows.size() != right_rows.size()) {
      LOG_WARN("LEFT/RIGHT transformed_rows size mismatch: L={}, R={}",
               left_rows.size(), right_rows.size());
    }

    for (size_t i = 0; i < min_rows; ++i)
      buffer << left_rows[i] << right_rows[i] << "\n";

  } catch (const std::out_of_range &e) {
    LOG_WARN("Missing LEFT or RIGHT in write_transformed: {}", e.what());
  }
}

void LidarSystemManager::write_track(
    std::ostringstream &buffer, const std::string &timestamp,
    const std::map<Props::Location, DataToPrint> &data) {

  try {
    const auto &left = data.at(Props::Location::LEFT);
    const auto &right = data.at(Props::Location::RIGHT);

    double height_diff =
        left.top_surface[2] - right.top_surface[2] + height_offset;
    double gauge =
        (left.side_surface - right.side_surface).norm() + width_offset;

    buffer << std::fixed << std::setprecision(6) << timestamp << ","
           << left.top_surface[0] << "," << left.top_surface[1] << ","
           << left.top_surface[2] << "," << right.top_surface[0] << ","
           << right.top_surface[1] << "," << right.top_surface[2] << ","
           << height_diff << "," << left.side_surface[0] << ","
           << left.side_surface[1] << "," << left.side_surface[2] << ","
           << right.side_surface[0] << "," << right.side_surface[1] << ","
           << right.side_surface[2] << "," << gauge << "\n";

  } catch (const std::out_of_range &e) {
    LOG_WARN("Missing LEFT or RIGHT in write_track: {}", e.what());
  }
}

void LidarSystemManager::merge_and_save(
    const std::string &folder, const std::string &timestamp,
    const std::map<Props::Location, DataToPrint> &data) {
  cv::Mat left_img = data.at(Props::Location::LEFT).disp_image;
  cv::Mat right_img = data.at(Props::Location::RIGHT).disp_image;

  // Convert to BGR if needed
  if (left_img.channels() == 1)
    cv::cvtColor(left_img, left_img, cv::COLOR_GRAY2BGR);
  if (right_img.channels() == 1)
    cv::cvtColor(right_img, right_img, cv::COLOR_GRAY2BGR);

  // Determine max width and height for uniform resizing
  int max_width = std::max(left_img.cols, right_img.cols);
  int max_height = std::max(left_img.rows, right_img.rows);
  cv::Size uniform_size(max_width, max_height);

  // Resize both to match
  cv::resize(left_img, left_img, uniform_size);
  cv::resize(right_img, right_img, uniform_size);

  // Label the images
  auto label = [](cv::Mat &img, const std::string &text) {
    cv::putText(img, text, cv::Point(img.cols - 120, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  };
  label(left_img, "LEFT");
  label(right_img, "RIGHT");

  // Create a vertical divider (5px wide, white)
  cv::Mat divider(uniform_size.height, 5, CV_8UC3, cv::Scalar(255, 255, 255));

  // Concatenate: LEFT | DIVIDER | RIGHT
  cv::Mat merged;
  cv::hconcat(std::vector<cv::Mat>{left_img, divider, right_img}, merged);

  // Save to file
  std::ostringstream name;
  name << folder << "/" << timestamp << ".png";

  try {
    cv::imwrite(name.str(), merged);
  } catch (const cv::Exception &e) {
    std::ostringstream error;
    error << "Failed to save image " << name.str() << ": " << e.what()
          << std::endl;
    error_log(error.str());
  }
}

std::string LidarSystemManager::build_header(const std::string &ip_left,
                                             const std::string &ip_right,
                                             const std::string &type) {
  std::ostringstream header;
  std::string data_header;

  if (type == "original") {
    data_header =
        std::string("Timestamps_left ") + "[" + Props::TIME_FMT + "]" +
        ",X_left [m],Z_left [m],Intensity_left,Peakwidth_left [m],"
        "Timestamps_right " +
        "[" + Props::TIME_FMT + "]" +
        ",X_right [m],Z_right [m],Intensity_right,Peakwidth_right [m]";
  } else if (type == "transformed") {
    data_header =
        std::string("Timestamps_left ") + "[" + Props::TIME_FMT + "]" +
        ",X_left [m],Y_left [m],Z_left [m],Intensity_left,Peakwidth_left [m],"
        "Timestamps_right " +
        "[" + Props::TIME_FMT + "]" +
        ",X_right [m],Y_right [m],Z_right [m],Intensity_right,Peakwidth_right "
        "[m]";
  } else if (type == "geometry") {
    data_header =
        std::string("Timestamps ") + "[" + Props::TIME_FMT + "]" +
        ",Top_x_left [m],Top_y_left [m],Top_z_left [m],"
        "Top_x_right [m],Top_y_right [m],Top_z_right [m],Height_diff[m],"
        "Side_x_left [m],Side_y_left [m],Side_z_left [m],"
        "Side_x_right [m],Side_y_right [m],Side_z_right [m],Track_guage[m]";
  } else {
    return "";
  }

  // Count number of columns by counting commas + 1
  int column_count = static_cast<int>(std::count(data_header.begin(),
                                                 data_header.end(), ',')) +
                     1;
  std::string padding(column_count - 1, ','); // one fewer commas than columns

  header << "# Lidar IP: " << ip_left << " Located on LEFT" << padding << "\n"
         << "# Lidar IP: " << ip_right << " Located on RIGHT" << padding << "\n"
         << data_header << "\n";

  return header.str();
}

void LidarSystemManager::write_batch(
    const std::string &orig_points_file,
    const std::string &transformed_points_file,
    const std::string &geom_points_file, const std::string &validation_folder,
    const std::map<Props::Location, DataToPrint> &new_info) {

  if (new_info.at(Props::Location::LEFT).time_data <
          params.init_complete_time ||
      new_info.at(Props::Location::RIGHT).time_data < params.init_complete_time)
    return;

  // Write into local buffers
  std::ostringstream local_orig, local_trans, local_geom;
  write_original(local_orig, new_info);
  write_transformed(local_trans, new_info);
  write_track(local_geom, new_info.begin()->second.timestamp_str, new_info);

  const std::string &name_left = new_info.at(Props::Location::LEFT).name;
  const std::string &name_right = new_info.at(Props::Location::RIGHT).name;

  // Write headers only if file does not exist
  if (!std::filesystem::exists(orig_points_file)) {
    std::ofstream(orig_points_file)
        << build_header(name_left, name_right, "original");
  }

  if (!std::filesystem::exists(transformed_points_file)) {
    std::ofstream(transformed_points_file)
        << build_header(name_left, name_right, "transformed");
  }

  if (!std::filesystem::exists(geom_points_file)) {
    std::ofstream(geom_points_file)
        << build_header(name_left, name_right, "geometry");
  }

  // Open the actual streams
  std::ofstream orig_points_stream(orig_points_file, std::ios::app);
  std::ofstream trans_points_stream(transformed_points_file, std::ios::app);
  std::ofstream geom_points_stream(geom_points_file, std::ios::app);

  if (!orig_points_stream || !trans_points_stream || !geom_points_stream) {
    error_log("Failed to open one or more output files.");
    return;
  }

  // Write content
  orig_points_stream << local_orig.str();
  trans_points_stream << local_trans.str();
  geom_points_stream << local_geom.str();

  // Compute average timestamp
  double avg_ts = 0.0;
  for (const auto &[_, data] : new_info)
    avg_ts += data.time_data;

  avg_ts /= static_cast<double>(new_info.size());

  // Save validation image
  merge_and_save(validation_folder, Props::format_timestamp(avg_ts), new_info);
}

bool LidarSystemManager::data_calibrated(
    const std::map<Props::Location, DataToPrint> &output_info) {
  {
    std::shared_lock<std::shared_mutex> lock(calibration_mutex);
    if (calibrated)
      return true;
  }

  std::unique_lock<std::shared_mutex> lock(calibration_mutex);
  if (calibrated)
    return true;

  ++calibrated_points;
  try {
    const auto &left = output_info.at(Props::Location::LEFT);
    const auto &right = output_info.at(Props::Location::RIGHT);

    avg_height += left.top_surface[2] - right.top_surface[2];
    avg_gauge += (left.side_surface - right.side_surface).norm();

  } catch (const std::out_of_range &e) {
    LOG_WARN("Missing LEFT or RIGHT in data calibration: {}", e.what());
    return false;
  }

  if (calibrated_points >= num_points_for_calibration) {
    calibrated = true;
    avg_height /= static_cast<double>(calibrated_points);
    avg_gauge /= static_cast<double>(calibrated_points);

    // calculating offsets
    height_offset = params.lidar.expected_height_deviation - avg_height;
    width_offset = params.lidar.expected_guage - avg_gauge;

    avg_height = avg_gauge = 0.0;
  }

  return false;
}

void LidarSystemManager::process_data(const TaskFuncs::Set &synced) {
  std::unordered_map<int, std::vector<LidarData::Ptr>> vals;
  size_t valid_num = 0;
  size_t min_count = std::numeric_limits<size_t>::max();
  double avg_ts = 0;

  for (const auto &lidar_info : synced) {
    auto data_opt = sensors[lidar_info.id]->get_data(lidar_info.timestamp);
    if (!data_opt || data_opt->empty())
      continue;

    ++valid_num;
    avg_ts += lidar_info.timestamp;

    min_count = std::min(min_count, data_opt->size());
    vals[lidar_info.id] = std::move(*data_opt);
  }

  if (valid_num != sensors.size()) {
    std::ostringstream warning;
    warning << "Partial sensor sync. Expected" << sensors.size() << " got "
            << valid_num << std::endl;
    warn_log(warning.str());
    return;
  }

  avg_ts /= static_cast<double>(valid_num);

  std::string orig_points_file, transformed_points_file, geom_points_file,
      validation_folder;
  {
    std::filesystem::path store_folder;
    double folder_ts = std::floor(avg_ts);
    store_folder = base_path / Props::format_timestamp(folder_ts);
    Props::ensure_dir(store_folder.string());
    Props::ensure_dir(store_folder.string() + "/Track Geometry Validation");

    orig_points_file = store_folder.string() + "/original.csv";
    transformed_points_file = store_folder.string() + "/transformed.csv";
    geom_points_file = store_folder.string() + "/track_geometry.csv";
    validation_folder = store_folder.string() + "/Track Geometry Validation";
  }

  for (size_t idx = 0; idx < min_count; ++idx) {
    // ------------------------------------------ //
    std::map<Props::Location, DataToPrint> output_info;
    for (size_t s_idx = 0; s_idx < sensors.size(); ++s_idx) {
      DataToPrint n_data(vals[s_idx][idx]);
      output_info[n_data.cnt_dir] = std::move(n_data);
    }

    if (output_info.empty())
      continue;

    if (data_calibrated(output_info))
      write_batch(orig_points_file, transformed_points_file, geom_points_file,
                  validation_folder, output_info);
  }
}
