/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: feature_matcher.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-30
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_

#include "pose_correction/modules/frame.h"
#include <iostream>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>

namespace pose_correction {
namespace modules {
class FeatureMatcher {
 public:
  typedef std::shared_ptr<FeatureMatcher> Ptr;

  FeatureMatcher() {}

  /**
   * match features, refine with histogram
   * TODO: 1. BF
   * */
  void MacthFeaturesBF(const modules::Frame::Ptr ref_frame,
                       const modules::Frame::Ptr cur_frame,
                       std::vector<cv::Point2f>& tracked_pts_ref,
                       std::vector<cv::Point2f>& tracked_pts_cur,
                       bool show_matches = false);

  /**
   * TODO: 2. Area based matches
   * */
  void MacthFeaturesArea(const modules::Frame::Ptr ref_frame,
                         const modules::Frame::Ptr cur_frame,
                         std::vector<cv::Point2f>& tracked_pts_ref,
                         std::vector<cv::Point2f>& tracked_pts_cur,
                         bool show_matches = false);

  /**
   * TODO: 3 LK flow
   * */
  void TrackFeaturesLK(const modules::Frame::Ptr ref_frame,
                                  const modules::Frame::Ptr cur_frame,
                                  std::vector<cv::Point2f>& tracked_pts_ref,
                                  std::vector<cv::Point2f>& tracked_pts_cur,
                                  bool show_trackes = false);

  /**
   * refine the matched points with histogram
   * */
  void CullWithHistConsistency(const std::vector<cv::KeyPoint>& keypoints_ref,
                               const std::vector<cv::KeyPoint>& keypoints_cur,
                               const std::vector<cv::DMatch>& raw_matches,
                               std::vector<cv::DMatch>& good_matches);
};
}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FEATURE_MATCHER_H_
