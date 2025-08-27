#include "sensors/manager.hpp"
#include <chrono>

DataManager::DataManager(FlowControl::Ptr ctrl) {

  // Dedicating 1 thread for the task processing cause
  tp = std::make_shared<TaskPool>(params.pool_threads);
  initialize(ctrl, tp);
}

DataManager::~DataManager() {
  if (tp)
    tp->shutdown();
}

TaskPool::Ptr DataManager::get_task_ptr() { return tp; }

void DataManager::create_storage_dir() {
  std::filesystem::path parent_dir = params.data_storage;

  // extract YYYYMMMDD
  auto [_, timestamp] = Props::create_timestamp();
  parent_dir /= timestamp.substr(0, timestamp.find('.'));

  try { // create directory if not existing already
    if (std::filesystem::create_directories(parent_dir)) {
      LOG_INFO("Directory created: {}", parent_dir.string());
    } else {
      LOG_INFO("Directory {} already exists", parent_dir.string());
    }

    // update the storage location
    params.data_storage = parent_dir.string();
  } catch (const std::filesystem::filesystem_error &e) {
    LOG_ERROR("Failed to create directory: {}, due to {}", parent_dir.string(),
              std::string(e.what()));
  }
}

void DataManager::initialize(FlowControl::Ptr ctrl, TaskPool::Ptr tp) {
  if (params.create_folders)
    create_storage_dir();

  // initialing sensors
  if (params.lidar.in_use)
    lm = LidarSystemManager::create(ctrl, tp);

  if (params.cam.in_use)
    cm = CamManager::create(ctrl, tp);

  if (params.temp.in_use)
    tm = ThermometerSystemManager::create(ctrl, tp);

  // ............... Ins Gets initialized last ............... //
  if (params.ins.in_use)
    im = InertialSystemManager::create(ctrl, tp);

  std::string init_comp_ts;
  std::tie(params.init_complete_time, init_comp_ts) = Props::create_timestamp();
  LOG_INFO("All Sensors Lodaed at: " + init_comp_ts + "!");
}
