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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_DATASET_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_DATASET_H_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace pose_correction {
namespace modules {
class Dataset {
 public:
  typedef std::shared_ptr<Dataset> Ptr;

  explicit Dataset(const std::string dataset_path)
      : dataset_path_(dataset_path) {
    trans_path_ = dataset_path_ + "/local_pose.csv";
    img_path_ = dataset_path_ + "/rgb";
  }

  bool GetAllImageNames(std::vector<cv::String>& all_image_names);

  bool GetAllImageNames(std::vector<std::string>& all_image_names);

  bool GetGroundTruthScale(std::vector<double>& all_trans);

  /**
   * NOTE: the index is same as the images' index
   * */
  bool ReadTranslation(const std::string filename, const int index,
                       Eigen::Vector3d* trans);

 private:
  std::string dataset_path_;
  std::string trans_path_;
  std::string img_path_;
};
}  // namespace modules
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_MODULES_DATASET_H_
