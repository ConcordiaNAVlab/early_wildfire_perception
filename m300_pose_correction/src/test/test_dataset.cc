/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_dataset.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-02
 *
 *   @Description:
 *
 *******************************************************************************/

#include "pose_correction/modules/dataset.h"
int main(int argc, char** argv) {
  pose_correction::modules::Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");

  std::vector<double> all_trans;
  dataset.GetGroundTruthScale(all_trans);
  std::vector<std::string> imgs;
  dataset.GetAllImageNames(imgs);
  for (size_t i = 0; i < imgs.size(); ++i) {
    std::cout << "scale:" << all_trans[i] << "img: " << imgs[i] << std::endl;
  }
  return 0;
}
