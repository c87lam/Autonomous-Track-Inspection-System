#ifndef SENSOR_CAMERA_FILTER_HPP
#define SENSOR_CAMERA_FILTER_HPP

#include "param.hpp"
#include <chrono>
#include <memory>
#include <opencv4/opencv2/features2d.hpp>

struct Feature {
  cv::Mat image, desc;
  std::vector<cv::KeyPoint> kp;
  bool valid = false;
};

// ---------------- Running Threshold ---------------------- //
struct RunningThreshold {
private:
  float ema_threshold = -1.0f; // Initial value (-1 means uninitialized)
  const float alpha;           // Smoothing factor

public:
  explicit RunningThreshold(float alpha_factor = 0.85f);

  // Update the running threshold using EMA
  void update(float new_threshold);

  // Get the current smoothed threshold
  float get() const;

  // Reset threshold (if needed)
  void reset();
};

// ---------------- Overlapping Threshold ---------------------- //
class OverlapCheck {
  Feature current_feat; // Previously called 'current'

  RunningThreshold thresh;
  cv::Ptr<cv::ORB> detector;
  bool initialized = false;

  std::chrono::steady_clock::time_point start_time;

public:
  using Ptr = std::shared_ptr<OverlapCheck>;

  OverlapCheck();
  explicit OverlapCheck(const cv::Mat &image);
  explicit OverlapCheck(Feature &feats);

  bool should_save(const cv::Mat &new_img);

private:
  void init_open_cv_stuff();
  Feature extractor(const cv::Mat &image);
  bool image_validation(const cv::Mat &image,
                        const std::chrono::steady_clock::time_point &curr_time);
};
#endif
