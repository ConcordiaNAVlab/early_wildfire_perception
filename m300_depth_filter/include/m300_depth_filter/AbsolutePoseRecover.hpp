/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: AbsolutePoseRecover.hpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-05
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_M300_DEPTH_FILTER_ABSOLUTEPOSERECOVER_HPP_
#define INCLUDE_M300_DEPTH_FILTER_ABSOLUTEPOSERECOVER_HPP_

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <sophus/se3.hpp>

namespace depth_filter {
/**
 * recover the absolute pose of the frame.
 * */
class AbsolutePoseRecover {
 public:
  AbsolutePoseRecover(const int beign_frame_index,
                      const std::string pose_filename,
                      const std::string abs_trans_filename)
      : ref_frame_index_(beign_frame_index),
        pose_filename_(pose_filename),
        abs_trans_filename_(abs_trans_filename) {}

  bool Recover();

 private:
  bool ReadPose(const std::string filename, const int line_index,
                int* frame_index, Sophus::SE3d* frame_pose);

  bool ReadTranslation(const std::string filename, const int frame_index,
                       Eigen::Vector3d* trans);
  // index where the pose is revcoverd.
  int ref_frame_index_{0};
  const std::string pose_filename_;
  const std::string abs_trans_filename_;
};
}  // namespace depth_filter

#endif  // INCLUDE_M300_DEPTH_FILTER_ABSOLUTEPOSERECOVER_HPP_
