#ifndef INS_ATIS_DATA_HELPER_HPP
#define INS_ATIS_DATA_HELPER_HPP
#include "sensors/adhoc.hpp"
#include <array>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <sstream>
#include <vector>

struct INSData {
  int id;
  double time;
  std::string timestamp;
  std::string name;

  // information
  std::string ins_data = "";

  using Ptr = std::shared_ptr<INSData>;
};

struct ExtractedData {
  double time;
  std::string timestamp;

  std::array<double, 3> rpy;         // roll pitch yaw
  std::array<double, 3> lat_lon_alt; // latitued longitude altitude
  std::array<double, 3> gyro;        // gyroscope data
  std::array<double, 3> acc;         // accelerometer data
  std::array<double, 3> mag;         // magnetometer data
  std::array<double, 2> speed;       // speed data

  double overall_speed;

  std::string csv_string() const;

  std::string formatted_string() const;
};

// ------------- Curvature Calculation ---------------- //
namespace Curvature {

enum class Type { STRAIGHT, CIRCULAR, CLOTHOID };

constexpr const char *to_string(Type type) {
  switch (type) {
  case Type::STRAIGHT:
    return "STRAIGHT";
  case Type::CIRCULAR:
    return "CIRCULAR";
  case Type::CLOTHOID:
    return "CLOTHOID";
  default:
    return "UNKNOWN";
  }
};

struct Info {
  std::string timestamp;
  Eigen::Vector3d lla_start, lla_end;
  Eigen::Vector2d xy_start, xy_end;
  double arc_length = 0.0;
  double chord_length = 0.0;
  double versine = 0.0;
  double mean_curvature = 0.0;
  double arc_window = 0.0;

  Type curve_type = Type::STRAIGHT;

  std::string csv_row() const;
};

// ---- Curve Tracker ---- //
class CurvatureTracker {
public:
  explicit CurvatureTracker(size_t max_size = 10);

  void reset();

  void add(double value);

  double get_mean() const;

  double get_std_dev() const;

  size_t size() const;

private:
  size_t max_size;
  std::deque<double> values;
  double mean;
  double M2;

  void remove_oldest();

  void insert_new(double value);
};

// ---- Arc Tracker ---- //
class ArcDevTracker {
  mutable std::shared_mutex data_mtx;
  std::vector<double> arc_lengths;

  struct Tracker {
    double arc_len = 0.0;
    double smoothed_curv = 0.0;
    double versine = 0.0;

    std::vector<Eigen::Vector2d> arc_path, curv_window;
    CurvatureTracker curve_info;

    std::ofstream file;
    double arc_window = 0.0;

    Tracker() : curve_info(10) {}

    void init_file(const std::string &folder, double arc_len_m,
                   const std::string &header);

    void reset();
  };

  std::vector<Tracker> trackers;

  Eigen::Vector3d origin_lla;
  Eigen::Vector3d ecef_origin;
  bool init = false;

  Eigen::Vector3d lla_start, lla_last;
  Eigen::Vector2d rhr_start, rhr_last;

  const Eigen::Matrix4d *transform_mat = nullptr;
  std::string output_folder;

public:
  using Ptr = std::shared_ptr<ArcDevTracker>;

  ArcDevTracker() = default;

  ArcDevTracker(const std::vector<double> &arc_lengths,
                const Eigen::Matrix4d &transform,
                const std::string &folder_path);

  static std::string get_csv_header();

  void init_origin(const std::vector<ExtractedData> &first_lla);

  void update(const ExtractedData &data);

private:
  Eigen::Vector3d lla2ecef(const Eigen::Vector3d &lla);

  Eigen::Vector3d ecef2rhr(const Eigen::Vector3d &ecef,
                           const Eigen::Vector3d &ecef_ref,
                           const Eigen::Vector3d &lla_ref);

  Eigen::Vector3d latlonalt2rhr(const Eigen::Vector3d &lla);

  void calculate_curvature(Tracker &tracker);

  void compute_versine(const Tracker &tracker, Info &seg);
};
}; // namespace Curvature
#endif
