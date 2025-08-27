#include "sensors/camera/filter.hpp"
#include <algorithm>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <queue>

namespace FilterCheck {
struct RunningMedian {
private:
  std::vector<float> all_values;
  std::priority_queue<float> lower_half;
  std::priority_queue<float, std::vector<float>, std::greater<float>>
      upper_half;

public:
  // Add a new number while maintaining balance
  void add(float value) {
    all_values.push_back(value);

    if (lower_half.empty() || value <= lower_half.top())
      lower_half.push(value);
    else
      upper_half.push(value);

    // Balance the two heaps
    if (lower_half.size() > upper_half.size() + 1) {
      upper_half.push(lower_half.top());
      lower_half.pop();
    } else if (upper_half.size() > lower_half.size()) {
      lower_half.push(upper_half.top());
      upper_half.pop();
    }
  }

  // Get the current median in O(1)
  float get_median() const {
    if (lower_half.empty())
      return 0.0f;

    return lower_half.size() > upper_half.size()
               ? lower_half.top()
               : (lower_half.top() + upper_half.top()) / 2.0f;
  }

  // Get Median Absolute Deviation (MAD)
  float get_mad() const {
    if (all_values.empty())
      return 0.0f; // Edge case: no elements

    float median = get_median();
    std::vector<float> deviations;
    deviations.reserve(all_values.size());

    // Compute absolute deviations from the median
    for (float v : all_values)
      deviations.push_back(std::abs(v - median));

    // Compute median of deviations
    size_t mid = deviations.size() / 2;
    std::nth_element(deviations.begin(), deviations.begin() + mid,
                     deviations.end());
    return deviations[mid];
  }

  void clear() {
    while (!lower_half.empty())
      lower_half.pop();
    while (!upper_half.empty())
      upper_half.pop();

    all_values.clear();
  }
};

static inline std::vector<cv::DMatch>
filter_top_matches(const std::vector<cv::DMatch> &matches, float ratio) {

  size_t count = std::max<size_t>(1, matches.size() * ratio);
  std::vector<cv::DMatch> sorted = matches;
  std::sort(sorted.begin(), sorted.end(),
            [](const cv::DMatch &a, const cv::DMatch &b) {
              return a.distance < b.distance;
            });
  return std::vector<cv::DMatch>(sorted.begin(), sorted.begin() + count);
}

static float compute_median_thresh(const std::vector<cv::DMatch> &matches) {
  RunningMedian running_median;
  for (const auto &m : matches)
    running_median.add(m.distance);

  float median_dist = running_median.get_median();
  float mad = running_median.get_mad();

  // Compute the new threshold using EMA
  return median_dist + 1.5 * mad;
}

static inline std::vector<cv::DMatch>
extract_good_ones(const std::vector<cv::DMatch> &matches, float threshold) {

  std::vector<cv::DMatch> good;
  for (const auto &m : matches)
    if (m.distance < threshold)
      good.push_back(m);
  return good;
}

static inline float compute_bbox_overlap(const std::vector<cv::KeyPoint> &kp1,
                                         const std::vector<cv::KeyPoint> &kp2) {

  std::vector<cv::Point2f> pts1, pts2;
  for (const auto &kp : kp1)
    pts1.push_back(kp.pt);
  for (const auto &kp : kp2)
    pts2.push_back(kp.pt);

  if (pts1.empty() || pts2.empty())
    return 0.0f;

  cv::Rect bbox1 = cv::boundingRect(pts1);
  cv::Rect bbox2 = cv::boundingRect(pts2);
  cv::Rect intersect = bbox1 & bbox2;
  cv::Rect union_box = bbox1 | bbox2;

  return static_cast<float>(intersect.area()) /
         static_cast<float>(union_box.area());
}

static inline float overlap_ratio(Feature &first, Feature &second,
                                  RunningThreshold &thresh) {
  // Brute force matcher more accurate here
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(first.desc, second.desc, matches);

  if (matches.empty())
    return 0.0f; // no match between the images so we update it

  { // update threshold value
    auto filtered = filter_top_matches(matches, 0.5f);
    float new_thresh = compute_median_thresh(filtered);
    thresh.update(new_thresh);
  }

  auto good_matches = extract_good_ones(matches, thresh.get());

  if (static_cast<int>(good_matches.size()) < 50)
    return compute_bbox_overlap(first.kp, second.kp);

  // Using Homography for accurate alignment
  std::vector<cv::Point2f> points1, points2;
  for (const auto &match : good_matches) {
    points1.push_back(first.kp[match.queryIdx].pt);
    points2.push_back(second.kp[match.trainIdx].pt);
  }

  // estimate homography
  std::vector<uchar> inliers;
  cv::Mat H = cv::findHomography(points1, points2, cv::RANSAC, 3.0, inliers);
  if (H.empty() || cv::countNonZero(inliers) < 10)
    return 0.0f;

  cv::Mat warmed;
  cv::warpPerspective(first.image, warmed, H, second.image.size(),
                      cv::INTER_NEAREST);

  // Count non-zero pixels for overlap calculation
  cv::Mat bin_warped, bin_second;
  cv::threshold(warmed, bin_warped, 30, 255, cv::THRESH_BINARY);
  cv::threshold(second.image, bin_second, 30, 255, cv::THRESH_BINARY);

  // Calculate intersection and union
  cv::Mat intersection, union_img;
  cv::bitwise_and(bin_warped, bin_second, intersection);
  cv::bitwise_or(bin_warped, bin_second, union_img);

  float intersect_count = static_cast<float>(cv::countNonZero(intersection));
  int union_count = cv::countNonZero(union_img);

  return (union_count == 0) ? 0.0f
                            : intersect_count / static_cast<float>(union_count);
}
}; // namespace FilterCheck

// ---------------- Running Threshold ---------------------- //
RunningThreshold::RunningThreshold(float alpha_factor) : alpha(alpha_factor) {}

// Update the running threshold using EMA
void RunningThreshold::update(float new_threshold) {
  if (ema_threshold < 0) // First update (initialize)
    ema_threshold = new_threshold;
  else
    ema_threshold = alpha * new_threshold + (1 - alpha) * ema_threshold;
}

// Get the current smoothed threshold
float RunningThreshold::get() const { return ema_threshold; }

// Reset threshold (if needed)
void RunningThreshold::reset() { ema_threshold = -1.0f; }

// ---------------- Overlapping Threshold ---------------------- //
OverlapCheck::OverlapCheck() {
  init_open_cv_stuff();
  start_time = std::chrono::steady_clock::now();
}

OverlapCheck::OverlapCheck(const cv::Mat &image) {
  init_open_cv_stuff();
  current_feat = extractor(image);
  start_time = std::chrono::steady_clock::now();
}

OverlapCheck::OverlapCheck(Feature &feats) {
  init_open_cv_stuff();
  current_feat = feats;
  start_time = std::chrono::steady_clock::now();
}

void OverlapCheck::init_open_cv_stuff() {
  if (initialized)
    return;

  detector = cv::ORB::create(500); // Limit keypoints for efficiency
  initialized = true;
}

Feature OverlapCheck::extractor(const cv::Mat &img) {
  Feature new_feat;

  cv::Mat gray;
  if (img.channels() > 1)
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else
    gray = img.clone();

  cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);
  cv::equalizeHist(gray, gray);

  new_feat.image = img.clone(); // Keep original for warping later
  detector->detectAndCompute(gray, cv::noArray(), new_feat.kp, new_feat.desc);
  new_feat.valid = !new_feat.desc.empty();

  return new_feat;
}

bool OverlapCheck::image_validation(
    const cv::Mat &new_img,
    const std::chrono::steady_clock::time_point &curr_time) {
  Feature new_feat = extractor(new_img);

  // Case 1: Invalid image
  if (!new_feat.valid) {
    thresh.reset();
    return false;
  }

  auto elapsed =
      std::chrono::duration_cast<Props::chrono_ms>(curr_time - start_time);
  if (elapsed >= params.cam.force_save) {
    current_feat = std::move(new_feat);
    start_time = curr_time;
    return true;
  }

  double overlap = FilterCheck::overlap_ratio(current_feat, new_feat, thresh);

  if (overlap > params.cam.overlap_ratio)
    return false;

  current_feat = std::move(new_feat);
  start_time = curr_time;

  if (overlap == 0.0)
    thresh.reset();

  return true;
}

bool OverlapCheck::should_save(const cv::Mat &new_img) {
  auto curr_time = std::chrono::steady_clock::now();

  if (!initialized)
    init_open_cv_stuff();

  // First valid image → always save
  if (!current_feat.valid) {
    current_feat = extractor(new_img);
    start_time = curr_time;
    return true;
  }

  return image_validation(new_img, curr_time);
}
