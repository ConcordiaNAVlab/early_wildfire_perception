/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: align_ir_rgb.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-23
 *
 *   @Description:
 *
 *******************************************************************************/

#include <Eigen/Core>

int main(int argc, char** argv) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;

  rot << 0.80892, 0.00365467, 109.598, -0.00703363, 0.800454, 45.114,
      -3.42839e-20, 3.37542e-19, 1;
  trans << 73.0345, -19.277, 6.88341e-16;

  // translate the rgb predicted fire point to the ir images



  return 0;
}
