#ifndef CAMERA_ATIS_DATA_HPP
#define CAMERA_ATIS_DATA_HPP

#include "filter.hpp"
#include "sensors/adhoc.hpp"
#include <memory>
#include <mutex>
#include <oneapi/tbb/concurrent_queue.h>
#include <tbb/concurrent_queue.h>
#include <unordered_map>
#include <unordered_set>

struct CameraData {
  using Ptr = std::shared_ptr<CameraData>;

  double time_data;
  std::string timestamp, name;
  Props::Location dir;
  cv::Mat image;
};

struct Boundaries {
  int64_t max_width;
  int64_t min_width;
  int64_t max_height;
  int64_t min_height;
};

// ----------------------------------- //
// ------- Individual Sensors -------- //
// ----------------------------------- //
namespace DataDevice {
class Camera : public BaseClass::Sensor {
  CameraDevice  *device = nullptr;
  CameraDataStream *data_stream = nullptr;

  OverlapCheck::Ptr oc = nullptr;

  std::vector<std::unique_ptr<CameraBuffer>> buffers;
  int def_width = 256, def_height = 256;
  int cv_type = -1;
  double scaling = 0.0;
  bool acquire_started = false, storage_val_updated = false;
  int storage_height, storage_width;

  Boundaries bounds;
  Props::Location dir;

public:
  using Ptr = std::unique_ptr<Camera>;

  Camera() = default;

  Camera(FlowControl::Ptr ctrl, int id, CameraDevice *device = nullptr,
         TaskPool::Ptr tp = nullptr);

  ~Camera();

  std::string get_ip(CameraDevice *_device);

  // ------------------ Initializers ------------------ //
  void initialize() override;

  // ------------------ Trigger controls ------------------ //
  void start() override;

  void pause() override;

  void stop() override;

  // ------------------ Data Processing and Alert ------------------ //
  void get_packet() override;

  std::optional<std::vector<CameraData::Ptr>> get_data(double timestamp);

private: // functions
  std::string create_name(CameraDevice *device);

  void configure();

  void get_default_size();

  void set_px_format();

  void set_gain_exposure();

  void data_stream_setup();

  void update_size();

  CameraData::Ptr process_data(CameraBuffer *buffer);

  void run();

private: // attributes
  std::unordered_map<double, std::vector<CameraData::Ptr>> storage;
};
} // namespace DataDevice

// ----------------------------------- //
// -------------- Manager ------------ //
// ----------------------------------- //
class CamManager : public BaseClass::Manager {
  static int camera_counter;
  std::vector<DataDevice::Camera::Ptr> cameras;
  std::vector<int> img_params;

  CameraSystem *system;
  CameraInterfaceList *ilist;
  CameraInterface *interface;
  CameraDeviceList *device_list;
  std::filesystem::path base_dir = params.data_storage + "/Images";

public:
  using Ptr = std::shared_ptr<CamManager>;

  CamManager(FlowControl::Ptr ctrl = nullptr, TaskPool::Ptr tp = nullptr);

  void initialize(FlowControl::Ptr ctrl = nullptr,
                  TaskPool::Ptr tp = nullptr) override;

  void process_data(const TaskFuncs::Set &synced) override;

  static Ptr create(FlowControl::Ptr ctrl, TaskPool::Ptr tp);

private:
  void initialize_img_params();

  int compression_scale(int scale, bool is_png);

  Ptr get_shared_data();
};

#endif
