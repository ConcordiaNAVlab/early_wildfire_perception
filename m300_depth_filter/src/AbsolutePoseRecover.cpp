/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: AbsolutePoseRecover.cpp
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
#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/FileWritter.hpp"

namespace depth_filter {
/**
 * find frame index in pose file, find corresponding translation in
 * translation file
 * */
bool AbsolutePoseRecover::Recover() {
  FileWritter pose_writer("abs_rel_pose.csv", 9);
  pose_writer.new_open();

  int frame_index = -1, last_frame_index = -1;
  for (int line_index = 0; line_index < 200; ++line_index) {
    Sophus::SE3d Tcw, Trw;
    Eigen::Vector3d abs_trans, abs_trans_ref;

    if (!ReadPose(pose_filename_, line_index, &frame_index, &Tcw)) return false;
    PRINT_INFO("line_index: %d, frame_index: %d", line_index, frame_index);

    // 1. not first
    // 2. not equal
    // 3 . bigger than given ref
    if (frame_index != last_frame_index + 1 && last_frame_index != -1 &&
        frame_index > ref_frame_index_) {
      PRINT_ERROR("frame is not consistent at line_index : %d", line_index);
      return false;
    }
    last_frame_index = frame_index;

    // found the line_index of the beginning frame
    if (frame_index < ref_frame_index_) {
      continue;
    } else if (frame_index == ref_frame_index_) {
      Trw = Tcw;
      ReadTranslation(abs_trans_filename_, ref_frame_index_, &abs_trans_ref);
      PRINT_INFO("ref_frame is at %d line", line_index);
      std::cout << "Trw" << Trw.rotationMatrix()
                << Trw.translation().transpose() << std::endl;
    }

    if (!ReadTranslation(abs_trans_filename_, frame_index, &abs_trans))
      return false;

    // NOTE: ref = world
    // recover realtive Trc
    double scale = (abs_trans - abs_trans_ref).norm();
    Sophus::SE3d Trc = Trw * Tcw.inverse();
    Trc.translation() = scale * Trc.translation().normalized();

    // wirte
    Eigen::Vector3d t = Trc.translation();
    Eigen::Matrix3d R = Trc.rotationMatrix();
    pose_writer.write(frame_index, R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0),
                      R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2));
  }
  return true;
}

bool AbsolutePoseRecover::ReadPose(const std::string filename,
                                   const int line_index, int* frame_index,
                                   Sophus::SE3d* frame_pose) {
  std::ifstream fin;
  fin.open(filename);
  if (!fin) {
    PRINT_ERROR("can not open %s in given path, no such file or directory!",
                filename.c_str());
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

  (*frame_pose) = Sophus::SE3d(q, t);

  return true;
}

bool AbsolutePoseRecover::ReadTranslation(const std::string filename,
                                          const int frame_index,
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
  depth_filter::SeekToLine(fin, frame_index + 1);
  // read each index, x, y, z, everytime
  for (int i = 0; i < 4; ++i) {
    if (!getline(fin, trans_tmp, ',')) {
      PRINT_ERROR("pose reading error! at index %d", frame_index);
      return false;
    }
    // PRINT_DEBUG("read trans:index+xyz:%.8f", std::stod(trans_tmp));
    trans_elements.push_back(std::stod(trans_tmp));
  }

  if (trans_elements[0] != static_cast<double>(frame_index)) {
    PRINT_INFO("mismach index of give and read! give: %d, read: %f",
               frame_index, trans_elements[0]);
    return false;
  }

  (*trans) << trans_elements[1], trans_elements[2], trans_elements[3];

  return true;
}

}  // namespace depth_filter
