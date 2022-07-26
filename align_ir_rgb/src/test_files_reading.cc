/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_files_reading.cc
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
#include <memory>

int main(int argc, char *argv[]) {
  std::vector<std::string> measurement_folder =
      GetSubFolders("/home/ls/align_images");

  std::vector<EachMeasurement::Ptr> all_measurements;
  for (std::string path_iter : measurement_folder) {
    EachMeasurement::Ptr measure = EachMeasurement::CreateFromFolder(path_iter);
    if (nullptr != measure) {
      all_measurements.push_back(measure);
      measure->PrintData();
    }
  }
}
