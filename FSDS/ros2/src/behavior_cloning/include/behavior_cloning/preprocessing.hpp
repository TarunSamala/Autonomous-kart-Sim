#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace behavior_cloning
{

inline float normalize_depth(
  const float depth_m, const float minimum_depth_m, const float maximum_depth_m)
{
  if (!(maximum_depth_m > minimum_depth_m)) {
    throw std::invalid_argument("maximum depth must exceed minimum depth");
  }
  if (!std::isfinite(depth_m) || depth_m < minimum_depth_m || depth_m > maximum_depth_m) {
    return 1.0F;
  }
  return std::clamp(
    (depth_m - minimum_depth_m) / (maximum_depth_m - minimum_depth_m), 0.0F, 1.0F);
}

inline cv::Mat align_depth_to_rgb(
  const cv::Mat & depth, const cv::Size & rgb_size,
  const double rgb_horizontal_fov_deg, const double depth_horizontal_fov_deg,
  const float missing_value)
{
  if (depth.empty() || depth.type() != CV_32FC1 || rgb_size.width <= 0 || rgb_size.height <= 0) {
    throw std::invalid_argument("invalid image passed to align_depth_to_rgb");
  }
  if (!(rgb_horizontal_fov_deg > 0.0 && rgb_horizontal_fov_deg < 180.0) ||
    !(depth_horizontal_fov_deg > 0.0 && depth_horizontal_fov_deg < 180.0))
  {
    throw std::invalid_argument("camera fields of view must be within (0, 180)");
  }

  constexpr double pi = 3.14159265358979323846;
  const double rgb_half_angle = rgb_horizontal_fov_deg * pi / 360.0;
  const double depth_half_angle = depth_horizontal_fov_deg * pi / 360.0;
  const double projection_scale = std::tan(depth_half_angle) / std::tan(rgb_half_angle);
  const int projected_width = std::clamp(
    static_cast<int>(std::lround(rgb_size.width * projection_scale)), 1, rgb_size.width);
  const int projected_height = std::clamp(
    static_cast<int>(std::lround(
      projected_width * static_cast<double>(depth.rows) / depth.cols)),
    1, rgb_size.height);

  cv::Mat resized;
  cv::resize(depth, resized, cv::Size(projected_width, projected_height), 0.0, 0.0, cv::INTER_NEAREST);
  cv::Mat aligned(rgb_size, CV_32FC1, cv::Scalar(missing_value));
  const int x = (rgb_size.width - projected_width) / 2;
  const int y = (rgb_size.height - projected_height) / 2;
  resized.copyTo(aligned(cv::Rect(x, y, projected_width, projected_height)));
  return aligned;
}

inline cv::Mat make_image_blob(
  const cv::Mat & bgr, const cv::Mat * depth_m, const int width, const int height,
  const float minimum_depth_m, const float maximum_depth_m,
  const double rgb_horizontal_fov_deg, const double depth_horizontal_fov_deg)
{
  if (bgr.empty() || bgr.type() != CV_8UC3 || width <= 0 || height <= 0) {
    throw std::invalid_argument("invalid RGB image or network dimensions");
  }
  cv::Mat resized_bgr;
  cv::resize(bgr, resized_bgr, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);
  cv::Mat rgb_float;
  cv::cvtColor(resized_bgr, rgb_float, cv::COLOR_BGR2RGB);
  rgb_float.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);

  std::vector<cv::Mat> channels;
  cv::split(rgb_float, channels);
  if (depth_m != nullptr) {
    cv::Mat aligned = align_depth_to_rgb(
      *depth_m, bgr.size(), rgb_horizontal_fov_deg, depth_horizontal_fov_deg,
      maximum_depth_m);
    cv::Mat resized_depth;
    cv::resize(aligned, resized_depth, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
    for (int row = 0; row < resized_depth.rows; ++row) {
      auto * values = resized_depth.ptr<float>(row);
      for (int column = 0; column < resized_depth.cols; ++column) {
        values[column] = normalize_depth(values[column], minimum_depth_m, maximum_depth_m);
      }
    }
    channels.push_back(resized_depth);
  }

  const int channel_count = static_cast<int>(channels.size());
  int dimensions[] = {1, channel_count, height, width};
  cv::Mat blob(4, dimensions, CV_32F);
  for (int channel = 0; channel < channel_count; ++channel) {
    cv::Mat destination(height, width, CV_32F, blob.ptr<float>(0, channel));
    channels[static_cast<std::size_t>(channel)].copyTo(destination);
  }
  return blob;
}

}  // namespace behavior_cloning
