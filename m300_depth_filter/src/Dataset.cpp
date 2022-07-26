/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: dataset.cc
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

#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/SystemLib.hpp"

#include "m300_depth_filter/Dataset.hpp"

namespace depth_filter {

bool Dataset::GetAllImageNames(const std::string img_path,
                               std::vector<std::string>* all_image_names,
                               const bool use_cv_global) {
  if (use_cv_global) {
    cv::glob(img_path, *all_image_names);
  } else {
    size_t image_num = GetFileNum(img_path);
    all_image_names->clear();
    all_image_names->reserve(image_num);
    for (size_t i = 0; i < image_num; ++i) {
      std::string img_name = img_path + "/" + std::to_string(i) + ".png";
      all_image_names->push_back(img_name);
    }
  }
  return true;
}

bool Dataset::ReadLocalPose(const std::string filename, const int index,
                            Eigen::Vector3d* trans) {
  std::ifstream fin;
  fin.open(filename);
  if (!fin) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                filename.c_str());
    return false;
  }

  std::string trans_tmp;
  std::vector<double> trans_elements;
  SeekToLine(fin, index + 1);
  // read each index, x, y, z, everytime
  for (int i = 0; i < 4; ++i) {
    if (!getline(fin, trans_tmp, ',')) {
      PRINT_ERROR("pose reading error! at index %d", index);
      return false;
    }
    // PRINT_DEBUG("read trans:index+xyz:%.8f", std::stod(trans_tmp));
    trans_elements.push_back(std::stod(trans_tmp));
  }

  if (trans_elements[0] != static_cast<double>(index)) {
    PRINT_INFO("mismach index of give and read! give: %d, read: %f", index,
               trans_elements[0]);
    return false;
  }

  (*trans) << trans_elements[1], trans_elements[2], trans_elements[3];

  fin.close();
  return true;
}

bool Dataset::ReadAbsRelPose(const int line_index, int* frame_index,
                             Sophus::SE3d* Twc) {
  std::ifstream fin;
  fin.open(abs_rel_pose_path_);
  if (!fin) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                abs_rel_pose_path_.c_str());
    return false;
  }
  std::string pose_tmp;
  std::vector<double> pose_elements;
  depth_filter::SeekToLine(fin, line_index);
  // read each index, x, y, z, everytime
  for (int i = 0; i < 13; ++i) {
    if (!getline(fin, pose_tmp, ',')) {
      PRINT_ERROR("pose reading error! at line_index %d", line_index);
      return false;
    }
    // PRINT_DEBUG("read trans:index+xyz:%.8f", std::stod(trans_tmp));
    pose_elements.push_back(std::stod(pose_tmp));
  }

  (*frame_index) = pose_elements[0];

  Eigen::Vector3d t;
  t << pose_elements[4], pose_elements[8], pose_elements[12];

  Eigen::Matrix3d R;
  R << pose_elements[1], pose_elements[2], pose_elements[3], pose_elements[5],
      pose_elements[6], pose_elements[7], pose_elements[9], pose_elements[10],
      pose_elements[11];
  // normalize it
  Eigen::Quaterniond q(R);
  q.normalize();

  (*Twc) = Sophus::SE3d(q, t);

  return true;
}

}  // namespace depth_filter
