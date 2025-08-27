#include "sensors/lidar/helper.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tbb/parallel_for_each.h>
#include <utility>
#include <vector>

namespace {
constexpr double MIN_CONTOUR_AREA = 5.0; // Minimum area for a valid contour
constexpr double dist_from_top = 0.015;  // meters from top of rail
constexpr double img_res = 0.001;        // meters per pixel

inline int extract_target(std::vector<std::vector<cv::Point>> contours,
                          Props::Location cnt_dir) {
  using ContourPair = std::pair<double, int>;
  using ContourPairVec = std::vector<ContourPair>;
  auto comp = [cnt_dir](const ContourPair &a, const ContourPair &b) {
    if (cnt_dir == Props::Location::LEFT)
      return a.first > b.first; // Min-heap for leftmost
    else
      return a.first < b.first; // Max-heap for rightmost
  };

  std::priority_queue<ContourPair, ContourPairVec, decltype(comp)> heap(comp);

  for (size_t idx = 0; idx < contours.size(); ++idx) {
    double contour_area = cv::contourArea(contours[idx]);
    if (contour_area < MIN_CONTOUR_AREA)
      continue;

    // Using the bounding box to determine this
    double x_pos = static_cast<double>(cv::boundingRect(contours[idx]).x);
    heap.emplace(x_pos, idx);
  }

  return heap.empty() ? -1 : heap.top().second;
}

std::pair<cv::Point, cv::Point> get_closest(std::vector<cv::Point> &contour,
                                            Props::Location dir) {

  size_t top_idx = 0;
  double max_x = Props::Location::RIGHT == dir
                     ? std::numeric_limits<double>::lowest()
                     : std::numeric_limits<double>::max();
  double mean_y = 0;

  for (size_t idx = 0; idx < contour.size(); ++idx) {
    bool gap_check = dir == Props::Location::RIGHT ? contour[idx].x > max_x
                                                   : contour[idx].x < max_x;
    if (gap_check) {
      max_x = contour[idx].x;
      top_idx = idx;
    }

    mean_y += contour[idx].y;
  }

  mean_y /= static_cast<double>(contour.size());
  double constant_shift =
      Props::Location::RIGHT == dir ? -dist_from_top : dist_from_top;
  double x_ref = contour[top_idx].x + constant_shift / img_res;

  // bottom surface
  size_t bttm_idx = 0;
  double y_dist = std::numeric_limits<double>::max();
  for (size_t idx = 0; idx < contour.size(); ++idx) {
    bool gap_check = dir == Props::Location::RIGHT
                         ? contour[idx].x < contour[top_idx].x
                         : contour[idx].x > contour[top_idx].x;

    if (gap_check && contour[idx].y < mean_y) {
      double dist = std::abs(contour[idx].x - x_ref);
      if (dist < y_dist) {
        y_dist = dist;
        bttm_idx = idx;
      }
    }
  }

  return {contour[top_idx], contour[bttm_idx]};
}
} // namespace

// ------------- PROFILE EXTRACTOR ----------------- //
int ProfilePrep::left_img_id = 0;
int ProfilePrep::right_img_id = 0;

void ProfilePrep::create_image() {
  double y_range = max_y - min_y;
  double z_range = max_z - min_z;

  int data_width = static_cast<int>(std::ceil(y_range / img_res)) + 1;
  int data_height = static_cast<int>(std::ceil(z_range / img_res)) + 1;

  int padding = 20; // pixels
  int img_width = data_width + 2 * padding;
  int img_height = data_height + 2 * padding;

  image = cv::Mat::zeros(img_height, img_width, CV_8UC1);
  pixel_to_point.clear();

  for (size_t idx = 0; idx < conv_points.rows(); ++idx) {
    const auto &pt = conv_points.row(idx);

    int col = static_cast<int>((pt[1] - min_y) / img_res) + padding;
    int row = img_height -
              (static_cast<int>((pt[2] - min_z) / img_res) + padding); // Flip Z

    if (col >= 0 && col < img_width && row >= 0 && row < img_height) {
      image.at<uchar>(row, col) = 255;
      pixel_to_point[{row, col}].push_back(idx);
    }
  }
}

bool ProfilePrep::surface_extractor(bool visualize, cv::Point &top_loc,
                                    cv::Point &side_loc, cv::Mat &disp_image) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat temp_image = image.clone();
  cv::findContours(temp_image, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty())
    return false;

  // Identify top of rail section
  int focus_id = extract_target(contours, cnt_dir);
  if (focus_id < 0)
    return false;

  // seperate top surface from side surface
  std::tie(top_loc, side_loc) = get_closest(contours[focus_id], cnt_dir);

  cv::Mat output;
  cv::cvtColor(image, output, cv::COLOR_GRAY2BGR);
  cv::drawContours(output, contours, -1, cv::Scalar(255, 255, 255), 1);
  cv::Scalar side_mark(0, 255, 0);  // Green
  cv::Scalar top_mark(0, 165, 255); // Orange

  cv::circle(output, side_loc, 3, side_mark, -1);
  cv::circle(output, top_loc, 3, top_mark, -1);

  if (cnt_dir == Props::Location::LEFT) {
    cv::rotate(output, output, cv::ROTATE_90_CLOCKWISE);
  } else {
    cv::rotate(output, output, cv::ROTATE_90_COUNTERCLOCKWISE);
  }

  // Draw colorblind-friendly legend in top-left
  const int lgnd_rad = 5;
  const int gap = 25;

  cv::circle(output, cv::Point(10, 20), lgnd_rad, side_mark, -1);
  cv::putText(output, "SIDE", cv::Point(20, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              side_mark, 1);

  cv::circle(output, cv::Point(10, 20 + gap), lgnd_rad, top_mark, -1);
  cv::putText(output, "TOP", cv::Point(20, 25 + gap), cv::FONT_HERSHEY_SIMPLEX,
              0.5, top_mark, 1);
  disp_image = output;

  if (visualize) {
    // Generate filename
    std::ostringstream img_name;
    if (cnt_dir == Props::Location::LEFT)
      img_name << "Left_side_lidar_" << left_img_id++ << ".png";
    else
      img_name << "Right_side_lidar_" << right_img_id++ << ".png";

    cv::imwrite(img_name.str(), output);
  }

  return true;
}

ProfilePrep::Result ProfilePrep::generate_surfaces(bool visualize) {
  cv::Point top, bottom;
  cv::Mat dis_image;
  ProfilePrep::Result res;

  if (!surface_extractor(visualize, top, bottom, dis_image))
    return res;

  const auto &top_vec = pixel_to_point[{top.y, top.x}];
  const auto &side_vec = pixel_to_point[{bottom.y, bottom.x}];

  if (top_vec.empty() || side_vec.empty())
    return res;

  res.top_surface.setZero();
  res.side_surface.setZero();

  for (const auto &idx : top_vec)
    res.top_surface.noalias() += conv_points.row(idx).transpose();

  for (const auto &idx : side_vec)
    res.side_surface.noalias() += conv_points.row(idx).transpose();

  // Get final results
  res.top_surface /= static_cast<double>(top_vec.size());
  res.side_surface /= static_cast<double>(side_vec.size());
  res.disp_image = dis_image;
  res.is_valid = true;

  return res;
}

ProfilePrep::Result ProfilePrep::surfaces(bool visualize) {
  create_image();
  return generate_surfaces(visualize);
}
