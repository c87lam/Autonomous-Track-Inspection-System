#ifndef LIDAR_ATIS_DATA_HPP
#define LIDAR_ATIS_DATA_HPP

#include "LidarSDK.h"
#include "helper.hpp"
#include <tbb/concurrent_queue.h>

struct LidarData {
  std::vector<std::string> orig_rows, transformed_rows;
  std::string timestamp, name;

  double time_data;

  ProfilePrep::Result contour;
  Props::Location cnt_dir;

  using Ptr = std::shared_ptr<LidarData>;
  using PtrVec = std::vector<Ptr>;

  // Functions
  struct Comparator {
    bool operator()(const Ptr &lhs, const Ptr &rhs) const {
      return lhs->time_data > rhs->time_data;
    }
  };
};

// Format for printing Data //
struct DataToPrint {
  double time_data;
  std::string timestamp_str;
  std::vector<std::string> orig_rows, transformed_rows;

  // Information for track stuff
  Eigen::Vector3d top_surface, side_surface;
  cv::Mat disp_image;

  Props::Location cnt_dir;
  std::string name;

  DataToPrint() = default;

  explicit DataToPrint(const LidarData::Ptr &data)
      : time_data(data->time_data), timestamp_str(data->timestamp),
        orig_rows(std::move(data->orig_rows)),
        transformed_rows(std::move(data->transformed_rows)),
        top_surface(std::move(data->contour.top_surface)),
        side_surface(std::move(data->contour.side_surface)),
        disp_image(std::move(data->contour.disp_image)), cnt_dir(data->cnt_dir),
        name(data->name) {}
};

namespace DataDevice {
class Lidar : public BaseClass::Sensor {
  std::atomic<int> queue_count{0};
  std::string ip;
  std::string port;

  bool acquire_started = false;
  void *scanner_handle;

  Eigen::Matrix4d transform_mat = Eigen::Matrix4d::Zero();
  Props::Location cntr_dir = Props::Location::NONE;

public:
  using Ptr = std::unique_ptr<Lidar>;

  Lidar() = default;

  Lidar(FlowControl::Ptr ctrl, const std::string &ip, const std::string &port,
        int id, TaskPool::Ptr tp = nullptr);

  ~Lidar();

  // initializing sensors
  void initialize() override;

  // connection to sensors e.t.c
  void get_packet() override;

  void start() override;

  void pause() override;

  void stop() override;

  void run();

  std::optional<std::vector<LidarData::Ptr>> get_data(double timestamp);

private:
  std::string create_name(const std::string &ip, const std::string &port);

  bool connection_status();

private: // attribute
  std::unordered_map<double, std::vector<LidarData::Ptr>> storage;
};
} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
class LidarSystemManager
    : public BaseClass::Manager,
      public std::enable_shared_from_this<LidarSystemManager> {
  std::vector<DataDevice::Lidar::Ptr> sensors;

  std::filesystem::path base_path;
  std::shared_mutex calibration_mutex;

  double last_batch_timestamp = 0.0;
  int calibrated_points = 0, num_points_for_calibration = 50;
  double height_offset = 0.0, width_offset = 0.0;
  double avg_height = 0.0, avg_gauge = 0.0;
  bool calibrated = false;

public:
  using Ptr = std::shared_ptr<LidarSystemManager>;

  LidarSystemManager(FlowControl::Ptr ctrl = nullptr,
                     TaskPool::Ptr tp = nullptr);

  ~LidarSystemManager();

  void initialize(FlowControl::Ptr ctrl = nullptr,
                  TaskPool::Ptr tp = nullptr) override;

  void process_data(const TaskFuncs::Set &synced) override;

  static Ptr create(FlowControl::Ptr ctrl, TaskPool::Ptr tp);

private:
  bool
  data_calibrated(const std::map<Props::Location, DataToPrint> &output_info);

  bool enough_data(const TaskFuncs::Set &synced);

  Ptr get_shared_data();

  std::string build_header(const std::string &ip_left,
                           const std::string &ip_right,
                           const std::string &type);

  void write_original(std::ostringstream &buffer,
                      const std::map<Props::Location, DataToPrint> &data);
  void write_transformed(std::ostringstream &buffer,
                         const std::map<Props::Location, DataToPrint> &data);

  void write_track(std::ostringstream &buffer, const std::string &timestamp,
                   const std::map<Props::Location, DataToPrint> &data);

  void merge_and_save(const std::string &folder, const std::string &timestamp,
                      const std::map<Props::Location, DataToPrint> &data);

  void write_batch(const std::string &orig_points_file,
                   const std::string &transformed_points_file,
                   const std::string &geom_points_file,
                   const std::string &validation_folder,
                   const std::map<Props::Location, DataToPrint> &new_info);
};
#endif
