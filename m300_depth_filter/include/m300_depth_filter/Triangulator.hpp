/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: Triangulation.hpp
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

#ifndef M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_TRIANGULATOR_HPP_
#define M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_TRIANGULATOR_HPP_

#include "m300_depth_filter/Frame.hpp"
#include <vector>

namespace depth_filter {
class Triangulator {
 public:
  void Run();

 private:
  void MacthFeaturesBF(const Frame& ref, const Frame& cur,
                       std::vector<cv::DMatch>* matches,
                       const bool use_hist = true);

  /**
   * refine the matched points with histogram
   * */
  void CullWithHistConsistency(const std::vector<cv::KeyPoint>& keypoints_ref,
                               const std::vector<cv::KeyPoint>& keypoints_cur,
                               const std::vector<cv::DMatch>& raw_matches,
                               std::vector<cv::DMatch>& good_matches);

  void Triangulation(const Sophus::SE3d T_ref, const Sophus::SE3d T_cur,
                     const std::vector<cv::Point2f>& pts_ref,
                     const std::vector<cv::Point2f>& pts_cur,
                     std::vector<Eigen::Vector3d>* points_3d);
};
}  // namespace depth_filter

#endif  // M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_TRIANGULATOR_HPP_
