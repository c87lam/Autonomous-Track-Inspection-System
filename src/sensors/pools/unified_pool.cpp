#include "sensors/pools/unified_pool.hpp"
#include <cstdlib>
#include <iomanip>

namespace {
inline double get_now() {
  auto [time, _] = Props::create_timestamp();
  return time;
}
} // namespace

Task::Task(Props::Sensors type, double timestamp, int sensor_id)
    : timestamp(timestamp), in_time(get_now()), type(type),
      sensor_id(sensor_id), severity(Props::sensor_priority(type)),
      state(TaskState::NeedSync) {}

Task::Task(double timestamp, Props::Sensors type, TaskFuncs::Set &&synced,
           int severity)
    : timestamp(timestamp), in_time(get_now()), type(type),
      severity(severity >= 0 ? severity : Props::sensor_priority(type)),
      synced_data(std::move(synced)), state(TaskState::Ready) {}

bool Task::Comparator::operator()(const Task &a, const Task &b) const {
  return a.severity == b.severity ? a.timestamp > b.timestamp
                                  : a.severity < b.severity;
}

// ---- Unified Pool ---- //
TaskPool::TaskPool(size_t thread_count) {
  if (thread_count < 1) {
    LOG_ERROR("Need at least 1 thread for task processing.");
    std::exit(EXIT_FAILURE);
  }

  for (size_t i = 0; i < thread_count; ++i)
    workers.emplace_back([this] { worker_loop(); });
}

TaskPool::~TaskPool() { shutdown(); }

void TaskPool::shutdown() {
  if (stop.exchange(true))
    return;
  LOG_INFO("Shutting Down Program. Completing Taks");
  {
    std::unique_lock<std::mutex> lock(queue_mutex);
    condition.notify_all();
  }

  for (auto &w : workers)
    if (w.joinable())
      w.join();
}

void TaskPool::enqueue(Props::Sensors type, double timestamp, int sensor_id) {
  if (stop.load())
    return;

  queue.emplace(type, timestamp, sensor_id);
  condition.notify_one();
}

void TaskPool::process_batch(std::vector<Task> &batch) {
  for (Task &task : batch) {
    if (task.state == TaskState::NeedSync) {
      auto it = updaters.find(task.type);
      if (it != updaters.end()) {
        if (auto res = it->second(task.sensor_id, task.timestamp)) {
          queue.emplace(task.timestamp, task.type, std::move(*res),
                        task.severity);
        }
      }
    } else if (task.state == TaskState::Ready) {
      auto it = consumers.find(task.type);
      if (it != consumers.end()) {
        LOG_INFO("Processing: " +
                 std::string(Props::sensors_to_string(task.type)) + " Data.");
        it->second(task.synced_data);
      } else {
        LOG_WARN("No consumer registered for " +
                 std::string(Props::sensors_to_string(task.type)));
      }
    }
  }
}

void TaskPool::worker_loop() {
  thread_local std::vector<Task> batch;
  thread_local TaskFuncs::BatchStats stats;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      condition.wait(lock, [this] { return !queue.empty() || stop; });
    }

    if (stop && queue.empty()) {
      LOG_INFO("Exiting Task Pool. Tasks Completed Data.");
      return;
    }

    batch.clear();
    std::unordered_set<Props::Sensors> seen;
    std::vector<Task> overflow;

    Task task;
    auto batch_start = hr_clock::now();
    double now = get_now();
    while ((int)batch.size() < stats.batch_size && queue.try_pop(task)) {
      auto wait_ms = Props::time_diff(now, task.in_time, Props::TimeScale::MS);

      if (wait_ms > Props::flush_threshold[task.type].count() &&
          task.severity < Props::sensor_priority(Props::Sensors::CAM)) {
        task.severity++;
        task.in_time = now;
        queue.push(std::move(task));
        continue;
      }

      if (seen.insert(task.type).second)
        batch.push_back(std::move(task));
      else
        overflow.push_back(std::move(task));
    }

    stats.update_duration(
        std::chrono::duration<double, std::milli>(hr_clock::now() - batch_start)
            .count());
    stats.adjust(queue.size());

    for (auto &extra : overflow)
      queue.push(std::move(extra));

    if (batch.empty())
      continue;

    std::sort(batch.begin(), batch.end(), [](const Task &a, const Task &b) {
      return (a.severity == b.severity) ? a.timestamp < b.timestamp
                                        : a.severity > b.severity;
    });

    process_batch(batch);
  }
}
