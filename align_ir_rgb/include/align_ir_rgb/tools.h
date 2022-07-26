/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: tools.h
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

#ifndef INCLUDE_ALIGN_IR_RGB_TOOLS_H_
#define INCLUDE_ALIGN_IR_RGB_TOOLS_H_

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Core>

/**
 * get sub folers name under given path
 * */
std::vector<std::string> GetSubFolders(const std::string path_name);

/**
 * locate the file to the line number
 * */
std::ifstream& SeekToLine(std::ifstream& in, const uint16_t line_nbr);

/**
 * data at each dir represents each measurement
 * */
struct EachMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  typedef std::shared_ptr<EachMeasurement> Ptr;

 public:
  std::string matches_file_path_;
  double distance_{0.0};
  std::array<Eigen::Vector3d, 4> homo_ir_pixel_pos_;
  std::array<Eigen::Vector3d, 4> homo_rgb_pixel_pos_;

 public:
  static bool GetOneHomePiexl(std::ifstream& in, Eigen::Vector3d* homo_pixel) {
    std::string pixel_tmp;

    for (int i = 0; i < 2; ++i) {
      if (!getline(in, pixel_tmp, ',')) {
        return false;
      }
      (*homo_pixel)[i] = std::stod(pixel_tmp);
    }

    (*homo_pixel)[2] = 1.0;
    return true;
  }

  /**
   * return nullptr if create failed!
   * */
  static EachMeasurement::Ptr CreateFromFolder(const std::string folder_name) {
    const std::string matches_file_name = folder_name + "/matches.txt";
    std::ifstream matches_fin;
    matches_fin.open(matches_file_name);
    if (!matches_fin) {
      std::cerr << "can not open " << matches_file_name
                << ", no such file or directory!" << std::endl;
      return nullptr;
    }

    // STEP: 1 read distance, ir, and rgb
    auto new_measurement = std::make_shared<EachMeasurement>();
    new_measurement->matches_file_path_ = matches_file_name;

    // line 0 -> distance
    std::string distance_tmp;
    if (!getline(matches_fin, distance_tmp)) return nullptr;
    new_measurement->distance_ = std::stod(distance_tmp);

    // line 1 -> ir data
    SeekToLine(matches_fin, 1);
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d homo_pixel;
      if (!GetOneHomePiexl(matches_fin, &homo_pixel)) return nullptr;
      new_measurement->homo_ir_pixel_pos_.at(i) = homo_pixel;
    }

    // line 2 -> rgb_data
    SeekToLine(matches_fin, 2);
    for (int i = 0; i < 4; ++i) {
      Eigen::Vector3d homo_pixel;
      if (!GetOneHomePiexl(matches_fin, &homo_pixel)) return nullptr;
      new_measurement->homo_rgb_pixel_pos_.at(i) = homo_pixel;
    }

    return new_measurement;
  }

  friend std::ostream& operator<<(std::ostream& out, const EachMeasurement& m);

  bool operator<(const EachMeasurement& other) {
    return (this->distance_ <= other.distance_);
  }

  void PrintData() const { std::cout << (*this); }
};

inline std::ostream& operator<<(std::ostream& out, const EachMeasurement& m) {
  out << "matches file path: " << m.matches_file_path_ << std::endl;
  out << "distace: " << m.distance_ << std::endl;
  for (int i = 0; i < 4; ++i) {
    out << "---------- " << i << " ----------" << std::endl;
    out << "ir^T: " << m.homo_ir_pixel_pos_.at(i).transpose() << std::endl;
    out << "rgb^T: " << m.homo_rgb_pixel_pos_.at(i).transpose() << std::endl;
  }

  return out;
}

#endif  // INCLUDE_ALIGN_IR_RGB_TOOLS_H_
