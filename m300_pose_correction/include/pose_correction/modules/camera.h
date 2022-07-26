/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: camera.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_

#include "pose_correction/modules/system_lib.h"

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace pose_correction {
namespace modules {
class Camera {
 public:
  typedef std::shared_ptr<Camera> Ptr;

  explicit Camera(const std::string camera_config_file)
      : file_path_(camera_config_file) {
    YAML::Node node = YAML::LoadFile(file_path_);
    fx_ = modules::GetParam<double>(node, "fx", 0.0f);
    fy_ = modules::GetParam<double>(node, "fy", 0.0f);
    cx_ = modules::GetParam<double>(node, "cx", 0.0f);
    cy_ = modules::GetParam<double>(node, "cy", 0.0f);

    K_ << fx_, 0.0f, cx_, 0.0f, fy_, cy_, 0.0f, 0.0f, 1.0f;
  }

 public:
  std::string file_path_;
  double fx_, fy_, cx_, cy_;
  Eigen::Matrix3d K_;
};
}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_CAMERA_H_
