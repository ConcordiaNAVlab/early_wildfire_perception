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

#include "pose_correction/modules/print_ctrl_macro.h"
#include "pose_correction/modules/system_lib.h"

#include "pose_correction/modules/dataset.h"

namespace pose_correction {
namespace modules {
bool Dataset::GetAllImageNames(std::vector<std::string>& all_image_names) {
  size_t image_num = modules::GetFileNum(img_path_);
  all_image_names.clear();
  all_image_names.reserve(image_num);
  for (size_t i = 0; i < image_num; ++i) {
    std::string img_name = img_path_ + "/" + std::to_string(i) + ".png";
    all_image_names.push_back(img_name);
  }
  return true;
}

bool Dataset::GetAllImageNames(std::vector<cv::String>& all_image_names) {
  cv::glob(img_path_, all_image_names);
  return true;
}

bool Dataset::GetGroundTruthScale(std::vector<double>& all_trans) {
  int i = 0;
  Eigen::Vector3d trans, prev_trans;
  while (ReadTranslation(trans_path_, i, &trans)) {
    Eigen::Vector3d d_t = trans - prev_trans;
    all_trans.push_back(d_t.norm());
    prev_trans = trans;
    ++i;
  }
  return true;
}

bool Dataset::ReadTranslation(const std::string filename, const int index,
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
  modules::SeekToLine(fin, index + 1);
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
}  // namespace modules
}  // namespace pose_correction
