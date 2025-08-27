#include "sensors/controller.hpp"

void FlowControl::ping_one() {
  std::unique_lock<std::mutex> lock(mtx);
  cv.notify_one();
}

void FlowControl::ping_all() {
  std::unique_lock<std::mutex> lock(mtx);
  cv.notify_all();
}

void FlowControl::start_spot_check() {
  spot_check_data.store(true, std::memory_order_relaxed);
  lat_long_spot_check.clear();
  run();
}

void FlowControl::stop_spot_check() {
  spot_check_data.store(false, std::memory_order_relaxed);
  pause();
}

Eigen::Vector3d FlowControl::get_location_data() {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  if (lat_long_spot_check.empty())
    return mean;

  for (const auto &val : lat_long_spot_check)
    mean.noalias() += val;

  mean /= static_cast<double>(lat_long_spot_check.size());
  lat_long_spot_check.clear();
  return mean;
}

void FlowControl::add_data(const double &lat, const double &lon,
                           const double &alt) {
  if (spot_check_data.load(std::memory_order_acquire))
    lat_long_spot_check.emplace_back(lat, lon, alt);
}

void FlowControl::run() {
  {
    std::unique_lock<std::shared_mutex> lock(stage_mtx);
    stage = Type::RUN;
  }

  set_state(Type::STOP, false);
  set_state(Type::RUN, true);
  ping_all();
}

void FlowControl::pause() {
  {
    std::unique_lock<std::shared_mutex> lock(stage_mtx);
    stage = Type::PAUSE;
  }

  set_state(Type::STOP, false);
  set_state(Type::RUN, false);
  ping_all();
}

void FlowControl::stop() {
  {
    std::unique_lock<std::shared_mutex> lock(stage_mtx);
    stage = Type::STOP;
  }

  set_state(Type::STOP, true);
  set_state(Type::RUN, false);
  ping_all();
}

bool FlowControl::trigger() const {
  // trigger happens when we need to run or stop
  bool running = run_att.load(std::memory_order_acquire);
  bool stopping = stop_att.load(std::memory_order_acquire);
  bool pausing = get_state(Type::PAUSE);

  return running || stopping || pausing;
}

void FlowControl::set_state(FlowControl::Type state, bool val) {
  if (stop_att.load(std::memory_order_relaxed))
    return; // Ensure we don't modify if stopped

  switch (state) {
  case Type::RUN:
    run_att.store(val, std::memory_order_relaxed);
    break;
  case Type::STOP:
    stop_att.store(val, std::memory_order_relaxed);
    break;
  default:
    break;
  }
}

bool FlowControl::get_state(FlowControl::Type state) const {
  switch (state) {
  case Type::RUN:
    return run_att.load(std::memory_order_acquire);
  case Type::STOP:
    return stop_att.load(std::memory_order_acquire);
  case Type::PAUSE:
    return !run_att.load(std::memory_order_acquire) &&
           !stop_att.load(std::memory_order_acquire);
  default:
    return false;
  }
}

FlowControl::Type FlowControl::current_stage() {
  std::shared_lock<std::shared_mutex> lock(stage_mtx);
  return stage;
}

void FlowControl::gui_trigger(bool forced) {
  forced_action.store(forced, std::memory_order_release);
}

bool FlowControl::forced_action_check() {
  return forced_action.load(std::memory_order_acquire);
}
