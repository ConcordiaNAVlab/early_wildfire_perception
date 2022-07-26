/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: dataset.h
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

#ifndef M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_
#define M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

namespace depth_filter {
class Dataset {
 public:
  typedef std::shared_ptr<Dataset> Ptr;

  explicit Dataset(const std::string dataset_path)
      : dataset_path_(dataset_path) {
    local_pose_path_ = dataset_path_ + "/local_pose.csv";
    rgb_img_path_ = dataset_path_ + "/rgb";
    rgb_seg_img_path_ = dataset_path_ + "/mask";
    ir_img_path_ = dataset_path_ + "/ir";
    abs_rel_pose_path_ = dataset_path_ + "/abs_rel_pose.csv";
  }

  bool GetAllRGBImageNames(std::vector<std::string>* all_image_names,
                           const bool use_cv_global = false) {
    return GetAllImageNames(rgb_img_path_, all_image_names, use_cv_global);
  }

  cv::Mat GetRGBImageByIndex(const int frame_index, const int type) {
    return cv::imread(
        rgb_img_path_ + "/" + std::to_string(frame_index) + ".png", type);
  }

  bool GetAllIRImageNames(std::vector<std::string>* all_image_names,
                          const bool use_cv_global = false) {
    return GetAllImageNames(ir_img_path_, all_image_names, use_cv_global);
  }

  cv::Mat GetIrImageByIndex(const int frame_index, const int type) {
    return cv::imread(ir_img_path_ + "/" + std::to_string(frame_index) + ".png",
                      type);
  }

  bool GetAllRGBSegImageNames(std::vector<std::string>* all_image_names,
                              const bool use_cv_global = false) {
    return GetAllImageNames(rgb_seg_img_path_, all_image_names, use_cv_global);
  }

  cv::Mat GetRGBSegImageByIndex(const int frame_index, const int type) {
    return cv::imread(
        rgb_seg_img_path_ + "/" + std::to_string(frame_index) + ".png", type);
  }

  bool ReadAbsRelPose(const int line_index, int* frame_index,
                      Sophus::SE3d* Twc);

 private:
  bool GetAllImageNames(const std::string img_folder,
                        std::vector<std::string>* all_image_names,
                        const bool use_cv_global = false);

  /**
   * NOTE: the index is same as the images' index
   * */
  bool ReadLocalPose(const std::string filename, const int frame_index,
                     Eigen::Vector3d* trans);

  std::string dataset_path_;
  std::string local_pose_path_;
  std::string rgb_img_path_;
  std::string rgb_seg_img_path_;
  std::string ir_img_path_;

  // 1. true scale
  // 2. the first frame is the ref frame
  std::string abs_rel_pose_path_;
};
}  // namespace depth_filter

#endif  // M300_DEPTH_FILTER_INCLUDE_M300_DEPTH_FILTER_DATASET_HPP_
