#include "behavior_cloning/preprocessing.hpp"

#include <limits>

#include <gtest/gtest.h>

TEST(Preprocessing, NormalizesAndRejectsInvalidDepth)
{
  EXPECT_FLOAT_EQ(behavior_cloning::normalize_depth(0.2F, 0.2F, 4.0F), 0.0F);
  EXPECT_FLOAT_EQ(behavior_cloning::normalize_depth(4.0F, 0.2F, 4.0F), 1.0F);
  EXPECT_FLOAT_EQ(behavior_cloning::normalize_depth(0.0F, 0.2F, 4.0F), 1.0F);
  EXPECT_FLOAT_EQ(
    behavior_cloning::normalize_depth(
      std::numeric_limits<float>::quiet_NaN(), 0.2F, 4.0F),
    1.0F);
}

TEST(Preprocessing, CentersNarrowerDepthFieldOfView)
{
  cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(1.0F));
  const auto aligned = behavior_cloning::align_depth_to_rgb(
    depth, cv::Size(640, 480), 80.9, 73.8, 4.0F);
  ASSERT_EQ(aligned.size(), cv::Size(640, 480));
  EXPECT_FLOAT_EQ(aligned.at<float>(240, 320), 1.0F);
  EXPECT_FLOAT_EQ(aligned.at<float>(240, 0), 4.0F);
  EXPECT_FLOAT_EQ(aligned.at<float>(240, 639), 4.0F);
}

TEST(Preprocessing, CreatesRgbAndRgbdBlobs)
{
  cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(2.0F));
  const auto rgb_blob = behavior_cloning::make_image_blob(
    rgb, nullptr, 160, 96, 0.2F, 4.0F, 80.9, 73.8);
  const auto rgbd_blob = behavior_cloning::make_image_blob(
    rgb, &depth, 160, 96, 0.2F, 4.0F, 80.9, 73.8);
  EXPECT_EQ(rgb_blob.size[1], 3);
  EXPECT_EQ(rgbd_blob.size[1], 4);
  EXPECT_EQ(rgbd_blob.size[2], 96);
  EXPECT_EQ(rgbd_blob.size[3], 160);
}
