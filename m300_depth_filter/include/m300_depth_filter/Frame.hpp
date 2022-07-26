/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: Frame.hpp
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

#ifndef M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_FRAME_HPP_
#define M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_FRAME_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace depth_filter {
class Frame {
 public:
  int id_;
  cv::Mat rgb_img_, ir_img_, rgb_seg_;
  Sophus::SE3d Twc_;

  // fire stuff
  cv::Mat descriptor_;
  std::vector<cv::KeyPoint> kps_;
  bool DetectFeatures();

 private:
  const float patch_width_{90.0f};
  const float patch_height_{90.0f};
};
}  // namespace depth_filter

#endif  // M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_FRAME_HPP_
