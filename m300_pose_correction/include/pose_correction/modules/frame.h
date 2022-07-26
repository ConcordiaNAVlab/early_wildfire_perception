/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_

#include <Eigen/Core>

#include <mutex>
#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace pose_correction {
namespace modules {
class Frame {
 public:
  typedef std::shared_ptr<Frame> Ptr;
  Frame() {
    Twc_ = Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  }

  Frame(const uint64_t id, const cv::Mat& img, const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t)
      : img_(img), id_(id), Twc_(Sophus::SE3d(R, t)) {}

  /**
   * 创建空的frame, 分配id
   * */
  static Frame::Ptr CreateFrame();

  /**
   * 提取特征点, 计算描述子
   * */
  bool DetectFeatures();

  /**
   * clear feature points
   * */
  void ClearFeatures() {
    kps_.clear();
    kps_pt_.clear();
  }

 public:
  cv::Mat img_;
  uint64_t id_;

  std::vector<cv::KeyPoint> kps_;
  std::vector<cv::Point2f> kps_pt_;
  cv::Mat descriptors_;

  Sophus::SE3d Twc_;
};

}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_FRAME_H_
