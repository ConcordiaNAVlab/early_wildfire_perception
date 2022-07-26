/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: Frame.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-07
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/Frame.hpp"
#include "m300_depth_filter/SegmentLocationFinder.hpp"

namespace depth_filter {
bool Frame::DetectFeatures() {
  cv::Mat binary_seg;
  cv::threshold(rgb_seg_, binary_seg, 250, 255, cv::THRESH_BINARY);

  std::vector<cv::Point2f> boundary_centers;
  std::vector<float> radiuses;
  depth_filter::SegmentLocationFinder finder;
  if (!finder.FindLocation(binary_seg, &boundary_centers, &radiuses, 5, false,
                           false)) {
    return false;
  }

  // detect in patch boxes
  cv::Mat patch_mask = cv::Mat::zeros(rgb_img_.size(), CV_8UC1);
  for (size_t i = 0; i < boundary_centers.size(); ++i) {
    cv::circle(patch_mask, boundary_centers[i], 20,
               cv::Scalar(255, 255, 255), -1);
  }

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  detector->detect(rgb_img_, kps_, patch_mask);
  detector->compute(rgb_img_, kps_, descriptor_);

  if (kps_.empty()) return false;

  return true;
}
}  // namespace depth_filter
