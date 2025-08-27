#include "sensors/ins/helper.hpp"
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace WGS_84 {
// conditions to transform lat-lon-alt -> enu frame

constexpr double a = 6378137.0;
constexpr double f = 1 / 298.257223563;
constexpr double e_sq = f * (2 - f);
} // namespace WGS_84

namespace Thresholds {
constexpr double versine = 0.01; // ~1 cm
constexpr double curv = 0.001;   // ~1 / 1000 m
constexpr double curv_std = 0.001;
} // namespace Thresholds

namespace Params {
constexpr size_t max_curv_history = 10;
constexpr double alpha = 0.25;
} // namespace Params

std::string ExtractedData::formatted_string() const {
  std::ostringstream oss;
  oss << "===================================\n";
  oss << "          GPS/IMU Information      \n";
  oss << "-----------------------------------\n";
  oss << std::left << std::setw(20) << "Timestamp[" << Props::TIME_FMT
      << "]:" << timestamp << "\n";
  oss << "-----------------------------------\n";
  oss << std::left << std::setw(20) << "Horizontal Speed:" << speed[0]
      << " m/s\n";
  oss << std::left << std::setw(20) << "Veritcal Speed:" << speed[1]
      << " m/s\n";
  oss << std::left << std::setw(20) << "Roll:" << rpy[0] << "\n";
  oss << std::left << std::setw(20) << "Pitch:" << rpy[1] << "\n";
  oss << std::left << std::setw(20) << "Yaw:" << rpy[2] << "\n";
  oss << std::left << std::setw(20) << "Latitude:" << lat_lon_alt[0] << "\n";
  oss << std::left << std::setw(20) << "Longitude:" << lat_lon_alt[1] << "\n";
  oss << std::left << std::setw(20) << "Altitude:" << lat_lon_alt[2] << " m\n";
  oss << std::left << std::setw(20) << "Gyroscope (X):" << gyro[0]
      << " rad/s\n";
  oss << std::left << std::setw(20) << "Gyroscope (Y):" << gyro[1]
      << " rad/s\n";
  oss << std::left << std::setw(20) << "Gyroscope (Z):" << gyro[2]
      << " rad/s\n";
  oss << std::left << std::setw(20) << "Accelerometer (X):" << acc[0]
      << " m/s²\n";
  oss << std::left << std::setw(20) << "Accelerometer (Y):" << acc[1]
      << " m/s²\n";
  oss << std::left << std::setw(20) << "Accelerometer (Z):" << acc[2]
      << " m/s²\n";
  oss << std::left << std::setw(20) << "Magnetometer (X):" << mag[0] << " µT\n";
  oss << std::left << std::setw(20) << "Magnetometer (Y):" << mag[1] << " µT\n";
  oss << std::left << std::setw(20) << "Magnetometer (Z):" << mag[2] << " µT\n";
  oss << "===================================\n";
  return oss.str();
}

std::string ExtractedData::csv_string() const {
  std::ostringstream oss;
  oss << timestamp << "," << speed[0] << "," << speed[1] << ",";
  oss << rpy[0] << "," << rpy[1] << "," << rpy[2] << ",";
  oss << lat_lon_alt[0] << "," << lat_lon_alt[1] << "," << lat_lon_alt[2]
      << ",";
  oss << gyro[0] << "," << gyro[1] << "," << gyro[2] << ",";
  oss << acc[0] << "," << acc[1] << "," << acc[2] << ",";
  oss << mag[0] << "," << mag[1] << "," << mag[2] << "\n";
  return oss.str();
}

// ------------- Curvature Calculation ---------------- //
namespace Curvature {

std::string Info::csv_row() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << timestamp << "," << lla_start[0]
      << "," << lla_start[1] << "," << lla_start[2] << "," << xy_start[0] << ","
      << xy_start[1] << "," << lla_end[0] << "," << lla_end[1] << ","
      << lla_end[2] << "," << xy_end[0] << "," << xy_end[1] << "," << versine
      << "," << arc_length << "," << chord_length << "," << mean_curvature
      << "," << arc_window << "," << Curvature::to_string(curve_type) << "\n";
  return oss.str();
}

// ---- Curve Tracker ---- //

CurvatureTracker::CurvatureTracker(size_t max_size)
    : max_size(max_size), mean(0.0), M2(0.0) {}

void CurvatureTracker::reset() {
  values.clear();
  mean = 0.0;
  M2 = 0.0;
}

void CurvatureTracker::add(double value) {
  if (values.size() == max_size) {
    remove_oldest();
  }
  insert_new(value);
}

double CurvatureTracker::get_mean() const { return mean; }

double CurvatureTracker::get_std_dev() const {
  if (values.size() <= 1)
    return 0.0;
  return std::sqrt(M2 / values.size());
}

size_t CurvatureTracker::size() const { return values.size(); }

void CurvatureTracker::remove_oldest() {
  double old = values.front();
  values.pop_front();

  double delta = old - mean;
  mean -= delta / max_size;
  M2 -= delta * (old - mean);
}

void CurvatureTracker::insert_new(double value) {
  double delta = value - mean;
  mean += delta / (values.size() + 1);
  M2 += delta * (value - mean);
  values.push_back(value);
}

// ---- Arc Tracker ---- //
void ArcDevTracker::Tracker::init_file(const std::string &folder,
                                       double arc_len_m,
                                       const std::string &header) {
  arc_window = arc_len_m;

  std::string path =
      folder + "/" + std::to_string(static_cast<int>(arc_len_m)) + ".csv";
  file.open(path, std::ios::out | std::ios::trunc);

  if (file.is_open()) {
    file << header;
  } else {
    throw std::runtime_error("Failed to open tracker file: " + path);
  }
}

void ArcDevTracker::Tracker::reset() {
  arc_len = 0.0;
  smoothed_curv = 0.0;
  versine = 0.0;
  arc_path.clear();
  curv_window.clear();
  curve_info.reset();
}

ArcDevTracker::ArcDevTracker(const std::vector<double> &arc_lengths,
                             const Eigen::Matrix4d &transform,
                             const std::string &folder_path)
    : arc_lengths(arc_lengths), transform_mat(&transform),
      output_folder(folder_path) {
  trackers.resize(arc_lengths.size());
  std::string header = get_csv_header();

  for (size_t idx = 0; idx < arc_lengths.size(); ++idx)
    trackers[idx].init_file(output_folder, arc_lengths[idx], header);
}

std::string ArcDevTracker::get_csv_header() {
  std::ostringstream oss;
  oss << "Timestamp, Lat_start, Lon_start, Alt_start, X_start [m], Y_start [m],"
      << " Lat_end, Lon_end, Alt_end, X_end [m], Y_end [m],"
      << " Versine [m], Arc_length [m], Chord_length [m], Mean_Curvature "
         "[1/m], Arc_Window [m], Curve_Type\n";
  return oss.str();
}

Eigen::Vector3d ArcDevTracker::lla2ecef(const Eigen::Vector3d &lla) {
  double lat = DEG2RAD(lla[0]);
  double lon = DEG2RAD(lla[1]);
  double alt = lla[2];
  double N =
      WGS_84::a / std::sqrt(1 - WGS_84::e_sq * std::sin(lat) * std::sin(lat));
  double x = (N + alt) * std::cos(lat) * std::cos(lon);
  double y = (N + alt) * std::cos(lat) * std::sin(lon);
  double z = ((1 - WGS_84::e_sq) * N + alt) * std::sin(lat);
  return {x, y, z};
}

Eigen::Vector3d ArcDevTracker::ecef2rhr(const Eigen::Vector3d &ecef,
                                        const Eigen::Vector3d &ecef_ref,
                                        const Eigen::Vector3d &lla_ref) {
  double lat0 = DEG2RAD(lla_ref[0]);
  double lon0 = DEG2RAD(lla_ref[1]);
  Eigen::Matrix3d R;
  R << -std::sin(lon0), std::cos(lon0), 0, -std::sin(lat0) * std::cos(lon0),
      -std::sin(lat0) * std::sin(lon0), std::cos(lat0),
      std::cos(lat0) * std::cos(lon0), std::cos(lat0) * std::sin(lon0),
      std::sin(lat0);
  Eigen::Vector3d enu = R * (ecef - ecef_ref);
  return transform_mat->block<3, 3>(0, 0) * enu;
}

Eigen::Vector3d ArcDevTracker::latlonalt2rhr(const Eigen::Vector3d &lla) {
  return ecef2rhr(lla2ecef(lla), ecef_origin, origin_lla);
}

void ArcDevTracker::init_origin(const std::vector<ExtractedData> &first_lla) {
  origin_lla.setZero();

  for (const auto &lla : first_lla) {
    origin_lla[0] += lla.lat_lon_alt[0];
    origin_lla[1] += lla.lat_lon_alt[1];
    origin_lla[2] += lla.lat_lon_alt[2];
  }

  origin_lla /= first_lla.size();
  ecef_origin = lla2ecef(origin_lla);
  init = true;
}

void ArcDevTracker::calculate_curvature(Tracker &tracker) {
  if (tracker.curv_window.size() > 3)
    tracker.curv_window.erase(tracker.curv_window.begin());

  if (tracker.curv_window.size() < 3)
    return;

  Eigen::Vector2d A = tracker.curv_window[0];
  Eigen::Vector2d B = tracker.curv_window[1];
  Eigen::Vector2d C = tracker.curv_window[2];

  Eigen::Vector2d v1 = (B - A).normalized();
  Eigen::Vector2d v2 = (C - B).normalized();

  double angle = std::acos(std::clamp(v1.dot(v2), -1.0, 1.0));
  // cross product
  double dir = v1.x() * v2.y() - v1.y() * v2.x();

  if (dir < 0)
    angle = -angle;
  double arc_local = (B - A).norm() + (C - B).norm();
  if (arc_local <= 1e-3)
    return;

  double kappa = angle / arc_local;
  tracker.smoothed_curv =
      Params::alpha * kappa + (1.0 - Params::alpha) * tracker.smoothed_curv;

  tracker.curve_info.add(kappa);
}

void ArcDevTracker::compute_versine(const Tracker &tracker, Info &seg) {
  // Compute chord midpoint
  Eigen::Vector2d chord_vec = rhr_last - rhr_start;
  Eigen::Vector2d chord_mid = chord_vec * 0.5;

  // Finding closest arc point
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector2d nearest_arc_pt;

  for (const auto &pt : tracker.arc_path) {
    double dist = (pt - chord_mid).squaredNorm();
    if (dist < min_dist) {
      min_dist = dist;
      nearest_arc_pt - pt;
    }
  }

  Eigen::Vector2d unit_chord = chord_vec.normalized();
  Eigen::Vector2d perp_dir(-unit_chord.y(), unit_chord.x());

  seg.chord_length = chord_vec.norm();
  seg.versine = std::abs(perp_dir.dot(nearest_arc_pt - chord_mid));
}

void ArcDevTracker::update(const ExtractedData &data) {
  if (!init)
    return;

  Eigen::Vector3d lla_eig = Eigen::Vector3d(
      data.lat_lon_alt[0], data.lat_lon_alt[1], data.lat_lon_alt[2]);
  Eigen::Vector2d current = latlonalt2rhr(lla_eig).head<2>();
  std::unique_lock<std::shared_mutex> lock(data_mtx);

  if (trackers.empty())
    return;

  if (trackers[0].arc_len == 0.0) {
    lla_start = lla_eig;
    rhr_start = current;
  }

  lla_last = lla_eig;
  rhr_last = current;

  for (size_t idx = 0; idx < arc_lengths.size(); ++idx) {
    auto &tracker = trackers[idx];
    double d_arc =
        (tracker.arc_path.empty() ? 0.0
                                  : (current - tracker.arc_path.back()).norm());
    tracker.arc_len += d_arc;
    tracker.arc_path.push_back(current);

    tracker.curv_window.push_back(current);
    calculate_curvature(tracker);

    if (tracker.arc_len < arc_lengths[idx])
      continue;

    Info seg;
    seg.timestamp = data.timestamp;
    seg.lla_start = lla_start;
    seg.lla_end = lla_last;
    seg.xy_start = rhr_start;
    seg.xy_end = rhr_last;
    seg.arc_length = tracker.arc_len;
    seg.mean_curvature = tracker.smoothed_curv;
    seg.arc_window = arc_lengths[idx];

    compute_versine(tracker, seg);
    double curv_std_dev = tracker.curve_info.get_std_dev();
    double mean_curv = tracker.smoothed_curv;

    if (mean_curv < Thresholds::curv && Thresholds::versine < 0.01) {
      seg.curve_type = Type::STRAIGHT;
    } else if (curv_std_dev < Thresholds::curv_std) {
      seg.curve_type = Type::CIRCULAR;
    } else {
      seg.curve_type = Type::CLOTHOID;
    }

    if (tracker.file.is_open())
      tracker.file << seg.csv_row();

    tracker.reset();
  }
}

} // namespace Curvature
