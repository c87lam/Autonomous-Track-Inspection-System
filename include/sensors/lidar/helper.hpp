#ifndef LIDAR_ATIS_DATA_HELPER_HPP
#define LIDAR_ATIS_DATA_HELPER_HPP

#include "sensors/adhoc.hpp"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <map>
#include <mutex>
#include <opencv4/opencv2/opencv.hpp>
#include <queue>

template <typename T, typename Compare = std::less<T>> class MinQueue {
private:
  std::priority_queue<T, std::vector<T>, Compare> pq;
  mutable std::mutex pq_mutex;

public:
  void push(const T &data) {
    std::lock_guard<std::mutex> lock(pq_mutex);
    pq.push(data);
  }

  T top() const {
    std::lock_guard<std::mutex> lock(pq_mutex);
    return pq.empty() ? nullptr : pq.top();
  }

  void pop() {
    std::lock_guard<std::mutex> lock(pq_mutex);
    if (!pq.empty())
      pq.pop();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(pq_mutex);
    return pq.empty();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(pq_mutex);
    return pq.size();
  }
};

// -------- Profile Extractor -------- //
struct ProfilePrep {
  struct Result {
    Eigen::Vector3d top_surface, side_surface;
    cv::Mat disp_image;
    bool is_valid = false;
  };

  static int left_img_id, right_img_id;

  void create_image();

  Result surfaces(bool save_img = false);

  Result generate_surfaces(bool visualize);

  bool surface_extractor(bool visualize, cv::Point &top, cv::Point &bottom,
                         cv::Mat &dis_image);

  // Rail coordinates we are only interested in x and y
  Eigen::MatrixXd conv_points;
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();

  // For Feature extraction
  cv::Mat image;
  std::map<std::pair<int, int>, std::vector<size_t>> pixel_to_point;
  Props::Location cnt_dir; // contour direction
};
#endif
