/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_each_measurement.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-18
 *
 *   @Description:
 *
 *******************************************************************************/

#include "align_ir_rgb/tools.h"
int main(int argc, char** argv) {
  std::string folder_name = "/home/ls/align_image_labeling_20220418/10.1";

  auto p_data = EachMeasurement::CreateFromFolder(folder_name);

  if (nullptr != p_data) p_data->PrintData();

  return 0;
}
