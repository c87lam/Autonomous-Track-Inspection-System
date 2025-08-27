#ifndef SENSOR_FLOW_CONTROLLER_HPP
#define SENSOR_FLOW_CONTROLLER_HPP

#include <atomic>
#include <condition_variable>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>
#include <mutex>
#include <oneapi/tbb/concurrent_vector.h>
#include <shared_mutex>
#include <tbb/concurrent_vector.h>

class FlowControl {
  std::atomic<bool> run_att{false};
  std::atomic<bool> stop_att{false};
  std::atomic<bool> forced_action{true};
  tbb::concurrent_vector<Eigen::Vector3d> lat_long_spot_check;
  std::atomic<bool> spot_check_data{false};
  mutable std::shared_mutex stage_mtx;

public:
  enum class Type { RUN, STOP, PAUSE, NONE, AUTO, MANUAL };

  using Ptr = std::shared_ptr<FlowControl>;

  FlowControl() = default;

  void run();

  void pause();

  void stop();

  bool trigger() const;

  bool get_state(Type state) const;

  void ping_one();

  void ping_all();

  void gui_trigger(bool forced = false);

  void add_data(const double &lat, const double &lon, const double &alt);

  Eigen::Vector3d get_location_data();

  bool forced_action_check();

  Type current_stage();

  void start_spot_check();

  void stop_spot_check();

public:
  std::condition_variable cv;
  std::mutex mtx;

private:
  void set_state(Type state, bool val);
  Type stage = Type::NONE;
};

#endif
