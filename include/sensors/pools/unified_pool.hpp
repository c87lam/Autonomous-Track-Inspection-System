#ifndef DATA_PROCESSING_POOL
#define DATA_PROCESSING_POOL

#include "param.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <tbb/concurrent_priority_queue.h>
#include <thread>
#include <unordered_set>
#include <vector>

namespace TaskFuncs {
struct Entry {
  int id;
  double timestamp;

  Entry(int id, double ts) : id(id), timestamp(ts) {}

  bool operator==(const Entry &other) const { return id == other.id; }
};

struct EntryHash {
  std::size_t operator()(const Entry &entry) const {
    return std::hash<int>{}(entry.id);
  }
};

using Set = std::unordered_set<Entry, EntryHash>;
// ------- Batch Statistics ------- //
struct BatchStats {
  int batch_size = 8;
  double avg_time_ms = 0;
  double smoothing = 0.1;

  void update_duration(double duration_ms) {
    avg_time_ms = smoothing * duration_ms + (1.0 - smoothing) * avg_time_ms;
  }

  void adjust(int queue_size) {
    if (avg_time_ms < 1.0 && queue_size > 16)
      batch_size = std::min(batch_size + 4, 64);

    else if (avg_time_ms > 10.0 || queue_size < 4)
      batch_size = std::max(batch_size - 2, 4);
  }
};
} // namespace TaskFuncs

enum class TaskState { NeedSync, Ready };
using hr_clock = std::chrono::high_resolution_clock;

// ---- Combining Tasks ---- //
struct Task {
  double timestamp, in_time;
  Props::Sensors type;
  int sensor_id, severity;
  TaskFuncs::Set synced_data;
  TaskState state;

  Task() = default;

  Task(Props::Sensors type, double timestamp, int sensor_id);

  Task(double timestamp, Props::Sensors type, TaskFuncs::Set &&synced,
       int severity = -1);

  struct Comparator {
    bool operator()(const Task &a, const Task &b) const;
  };
};

// ---- Unified Pool ---- //
class TaskPool {
public:
  using Ptr = std::shared_ptr<TaskPool>;
  TaskPool(size_t thread_count = params.pool_threads);

  ~TaskPool();

  template <typename Manager>
  void register_manager(Props::Sensors type, std::shared_ptr<Manager> mgr) {
    updaters[type] = [mgr](int id, double ts) {
      return mgr->update_data_count(id, ts);
    };
    consumers[type] = [mgr](const TaskFuncs::Set &data) {
      mgr->process_data(data);
    };
  }

  void enqueue(Props::Sensors type, double timestamp, int sensor_id);

  void shutdown();

private:
  using UpdateFunc = std::function<std::optional<TaskFuncs::Set>(int, double)>;
  using ProcessFunc = std::function<void(const TaskFuncs::Set &)>;

  std::vector<std::thread> workers;
  tbb::concurrent_priority_queue<Task, Task::Comparator> queue;
  std::unordered_map<Props::Sensors, UpdateFunc> updaters;
  std::unordered_map<Props::Sensors, ProcessFunc> consumers;

  std::mutex queue_mutex;
  std::condition_variable_any condition;
  std::atomic<bool> stop{false};

  void process_batch(std::vector<Task> &batch);
  void worker_loop();
};

#endif
