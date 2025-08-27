#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP

#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Timestamp {

template <typename dataPtr> struct MinHeapCompare {
  bool operator()(const std::pair<int, dataPtr> &a,
                  const std::pair<int, dataPtr> &b) const {
    return a.first > b.first;
  }
};

template <typename dataPtr>
using HeapType = std::priority_queue<std::pair<int, dataPtr>,
                                     std::vector<std::pair<int, dataPtr>>,
                                     MinHeapCompare<dataPtr>>;

template <typename dataPtr>
void matchTimestamps(int id, const std::vector<dataPtr> &sensor,
                    std::vector<dataPtr> &min_data, std::unordered_map<double, HeapType<dataPtr>> &paired_info) {

    struct MatchCandidate {
        size_t min_idx;
        size_t sensor_idx;
        double diff;
        bool operator<(const MatchCandidate &other) const {
            return diff < other.diff;
        }
    };
    using Val = std::pair<double, dataPtr>;
    double median_min = min_data[min_data.size() / 2]->time_data;
    double median_sensor = sensor[sensor.size() / 2]->time_data;
    double tolerance = std::abs(median_min - median_sensor)*2;

    std::vector<MatchCandidate> candidates;
    for (std::size_t i = 0; i < min_data.size(); ++i) {
        double t = min_data[i]->time_data;

        auto it = std::lower_bound(sensor.begin(), sensor.end(), t, [](const dataPtr &a, double val) {
            return a->time_data < val;
        });

        if (it != sensor.end()) {
            std::size_t j = it - sensor.begin();
            double diff = std::abs(sensor[j]->time_data - t);
            if (diff <= tolerance){
                candidates.push_back({i, j, diff});
            }
        }

        if (it != sensor.begin()) {
            std::size_t j = (it - sensor.begin()) - 1;
            double diff = std::abs(sensor[j]->time_data - t);
            if (diff <= tolerance){
                candidates.push_back({i, j, diff});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end());
    std::vector<bool> min_used(min_data.size(), false);
    std::vector<bool> sensor_used(sensor.size(), false);

    for (const auto &match : candidates) {
        if (min_used[match.min_idx] || sensor_used[match.sensor_idx]){
            continue;
        }

        double t = min_data[match.min_idx]->time_data;
        paired_info[t].emplace(id, sensor[match.sensor_idx]);

        min_used[match.min_idx] = true;
        sensor_used[match.sensor_idx] = true;
    }

    for (std::size_t i=0; i<min_data.size(); ++i) {
        if (min_used[i]){ 
            continue;
        }

        double t = min_data[i]->time_data;
        double best_diff = std::numeric_limits<double>::infinity();
        std::size_t best_idx = -1;
        
        for (std::size_t j = 0; j<sensor.size(); ++j) {
            if (sensor_used[j]){
                continue;
            }
            double diff = std::abs(sensor[j]->time_data - t);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = j;
            }
        }

        if (best_idx != size_t(-1)) {
            paired_info[t].emplace(id, sensor[best_idx]);
            min_used[i] = true;
            sensor_used[best_idx] = true;
        }
    }
}
      
template <typename dataPtr>
void printTimestamps(std::vector<dataPtr> &min_data,
                     std::unordered_map<double, HeapType<dataPtr>> &paired_info,
                     std::ofstream &output) {

  std::ostringstream out;
  for (size_t i = 0; i < min_data.size(); i++) {
    auto iter = paired_info.find(min_data[i]->time_data);
    out << min_data[i]->timestamp << ", ";

    while (!iter->second.empty()) {
      const auto &data_ptr = iter->second.top();
      iter->second.pop();
      out << "," << data_ptr.second->timestamp << ", ";
    }
    out << "\n";
  }
  output << out.str();
}

} // namespace Timestamp

#endif