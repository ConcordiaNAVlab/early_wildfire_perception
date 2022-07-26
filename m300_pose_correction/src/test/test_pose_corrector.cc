/*******************************************************************************
*   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
*
*   @Filename: test_pose_corrector.cc
*
*   @Author: Shun Li
*
*   @Email: 2015097272@qq.com
*
*   @Date: 2022-06-01
*
*   @Description: 
*
*******************************************************************************/


#include "pose_correction/app/pose_corrector.h"

int main(int argc, char** argv) {
  pose_correction::app::PoseCorrector calculator;
  calculator.Run();
  return 0;
}
