/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_feature_matcher.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-01
 *
 *   @Description:
 *
 *******************************************************************************/

#include <vector>
#include "pose_correction/modules/feature_matcher.h"
#include "pose_correction/modules/frame.h"

int main(int argc, char** argv) {
  pose_correction::modules::FeatureMatcher matcher;

  for (int i = 0; i < 100; ++i) {
    cv::Mat img_ref = cv::imread(
        "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_51.2/"
        "rgb/"
        +std::to_string(i)+
        ".png",
        cv::IMREAD_GRAYSCALE);
    auto ref_frame = pose_correction::modules::Frame::CreateFrame();
    ref_frame->img_ = img_ref;
    ref_frame->DetectFeatures();

    cv::Mat img_cur = cv::imread(
        "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_51.2/"
        "rgb/"
        +std::to_string(i+1)+
        ".png",
        cv::IMREAD_GRAYSCALE);
    auto cur_frame = pose_correction::modules::Frame::CreateFrame();
    cur_frame->img_ = img_cur;
    cur_frame->DetectFeatures();

    std::vector<cv::Point2f> matched_pts_ref, matched_pts_cur;
    matcher.MacthFeaturesBF(ref_frame, cur_frame, matched_pts_ref,
                            matched_pts_cur, true);
  }

  return 0;
}
