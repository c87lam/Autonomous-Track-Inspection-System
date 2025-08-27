#ifndef DATA_MANAGER_HPP
#define DATA_MANAGER_HPP

#include "param.hpp"
#include "sensors/camera/func.hpp"
#include "sensors/ins/func.hpp"
#include "sensors/lidar/func.hpp"
#include "sensors/pools/unified_pool.hpp"
#include "sensors/thermometer/func.hpp"

class DataManager {
  TaskPool::Ptr tp = nullptr;

  LidarSystemManager::Ptr lm = nullptr;

  CamManager::Ptr cm = nullptr;

  InertialSystemManager::Ptr im = nullptr;

  ThermometerSystemManager::Ptr tm = nullptr;

public:
  using Ptr = std::shared_ptr<DataManager>;

  DataManager(FlowControl::Ptr ctrl = nullptr);

  ~DataManager();

  TaskPool::Ptr get_task_ptr();

private:
  void create_storage_dir();

  void initialize(FlowControl::Ptr ctrl, TaskPool::Ptr tp);
};
#endif
