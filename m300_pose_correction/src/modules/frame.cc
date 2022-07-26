/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: frame.cc
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-29
 *
 *   @Description:
 *
 *******************************************************************************/

#include "pose_correction/modules/frame.h"
#include "pose_correction/modules/ORBextractor.h"

namespace pose_correction {
namespace modules {

bool Frame::DetectFeatures() {
  if (img_.empty()) {
    std::cerr << "empty image!" << std::endl;
    return false;
  }

  auto orb_extractor_ptr =
      std::make_shared<ORB::ORBextractor>(1000, 1.2, 8, 20, 7);
  orb_extractor_ptr->operator()(img_, cv::Mat(), kps_, descriptors_);

  cv::KeyPoint::convert(kps_, kps_pt_);

  return true;
}

Frame::Ptr Frame::CreateFrame() {
  static uint64_t factory_id;
  Frame::Ptr new_frame(new Frame);
  new_frame->id_ = factory_id++;
  return new_frame;
}

}  // namespace modules
}  // namespace pose_correction
