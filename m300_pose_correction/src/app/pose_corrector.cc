/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: pose_corrector.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-31
 *
 *   @Description:
 *
 *******************************************************************************/

#include "pose_correction/app/pose_corrector.h"
#include <memory>
#include <opencv2/core/eigen.hpp>

namespace pose_correction {
namespace app {

PoseCorrector::PoseCorrector() {
  dataset_ = std::make_shared<modules::Dataset>(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");
  camera_ = std::make_shared<modules::Camera>(
      "/home/ls/m300_depth_filter/H20T_visiable_calbi/H20T_visible_param.yaml");
  viewer_ = std::make_shared<modules::Viewer>();

  feat_matcher_ = std::make_shared<modules::FeatureMatcher>();
}

void PoseCorrector::Run() {
  std::vector<std::string> imgs_paths;
  dataset_->GetAllImageNames(imgs_paths);
  std::vector<double> frame_scales;
  dataset_->GetGroundTruthScale(frame_scales);

  if (imgs_paths.empty() || frame_scales.empty() ||
      (imgs_paths.size() != frame_scales.size())) {
    std::cerr << "读取数据集有误!" << std::endl;
    std::cerr << "图像数据非空:" << imgs_paths.empty() << std::endl;
    std::cerr << "轨迹数据非空:" << frame_scales.empty() << std::endl;
    std::cerr << "图像与轨迹数相等:"
              << (imgs_paths.size() != frame_scales.size()) << std::endl;
    return;
  }

  // first frame
  auto ref_frame = modules::Frame::CreateFrame();
  ref_frame->img_ = cv::imread(imgs_paths[0], cv::IMREAD_GRAYSCALE);
  ref_frame->DetectFeatures();

  // trajectory
  modules::Viewer::TrajType traj;

  for (size_t i = 1; i < imgs_paths.size(); ++i) {
    // 初始化当前帧
    auto cur_frame = modules::Frame::CreateFrame();
    cur_frame->img_ = cv::imread(imgs_paths[i], cv::IMREAD_GRAYSCALE);
    cur_frame->DetectFeatures();

    std::cout << " current img: " << imgs_paths[i] << std::endl;

    // feature maching
    std::vector<cv::Point2f> matched_pts_ref;
    std::vector<cv::Point2f> matched_pts_cur;
    feat_matcher_->MacthFeaturesBF(ref_frame, cur_frame, matched_pts_ref,
                                   matched_pts_cur, true);

    // 对极几何更新当前帧的位姿
    EstimatePose(matched_pts_ref, matched_pts_cur, frame_scales[i], ref_frame,
                 cur_frame);

    // 将当前帧位姿存入, 更新显示
    if (viewer_) {
      traj.push_back(cur_frame->Twc_);
      viewer_->Update(traj);
    }

    // 将当前帧作为为下一帧的参考帧
    ref_frame = cur_frame;
  }
}

void PoseCorrector::EstimatePose(const std::vector<cv::Point2f>& pts_ref,
                                 const std::vector<cv::Point2f>& pts_cur,
                                 const double scale,
                                 const modules::Frame::Ptr ref_frame,
                                 modules::Frame::Ptr cur_frame) {
  cv::Mat E, R_rc, t_rc, K;
  cv::eigen2cv(camera_->K_, K);

  E = cv::findEssentialMat(pts_cur, pts_ref, K, cv::RANSAC);
  cv::recoverPose(E, pts_cur, pts_ref, K, R_rc, t_rc);

  std::cout << "delta rotationMatrix: \n" << R_rc << std::endl;
  std::cout << "delta translation: \n" << t_rc << std::endl;

  Eigen::Matrix3d eigen_R_rc;
  Eigen::Vector3d eigen_t_rc;
  cv::cv2eigen(R_rc, eigen_R_rc);
  cv::cv2eigen(t_rc, eigen_t_rc);

  Sophus::SE3d Trc = Sophus::SE3d(eigen_R_rc, scale * eigen_t_rc);

  cur_frame->Twc_ = ref_frame->Twc_ * Trc;

  // std::cout << "translation: \n" << cur_frame->Twc_.translation() <<
  // std::endl; std::cout << "rotationMatrix: \n" <<
  // cur_frame->Twc_.rotationMatrix()
  //           << std::endl;
}

}  // namespace app
}  // namespace pose_correction
