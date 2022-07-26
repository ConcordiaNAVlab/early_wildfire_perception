/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: SegmentLocationFinder.hpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-16
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_MODULES_DEPTHESTIMATOR_SEGMENTLOCATIONFINDER_HPP_
#define INCLUDE_MODULES_DEPTHESTIMATOR_SEGMENTLOCATIONFINDER_HPP_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

namespace depth_filter {

class SegmentLocationFinder {
 public:
  bool FindLocation(const cv::Mat binary_input,
                    std::vector<cv::Point2f>* centers,
                    std::vector<float>* radius, const int morph_size,
                    const bool imshow_contours, const bool imshow_final_rect);
};
}  // namespace depth_filter

#endif  // INCLUDE_MODULES_DEPTHESTIMATOR_SEGMENTLOCATIONFINDER_HPP_
