/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_AbsolutePoseRecover.cpp
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

#include "m300_depth_filter/AbsolutePoseRecover.hpp"
int main(int argc, char** argv) {
  depth_filter::AbsolutePoseRecover recover(
      40,
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_51.2/"
      "Monoslam_pose.csv",
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_51.2/"
      "local_pose.csv");

  recover.Recover();
  return 0;
}
