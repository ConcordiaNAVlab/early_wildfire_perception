/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: pose_corrector.h
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

#ifndef M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_APP_POSE_CORRECTOR_H_
#define M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_APP_POSE_CORRECTOR_H_

#include "pose_correction/modules/dataset.h"
#include "pose_correction/modules/camera.h"
#include "pose_correction/modules/viewer.h"
#include "pose_correction/modules/frame.h"
#include "pose_correction/modules/feature_matcher.h"

#include <vector>

namespace pose_correction {
namespace app {
class PoseCorrector {
 public:
  PoseCorrector();

  void Run();

 private:
  void EstimatePose(const std::vector<cv::Point2f>& pts_ref,
                    const std::vector<cv::Point2f>& pts_cur, const double scale,
                    const modules::Frame::Ptr ref_frame,
                    modules::Frame::Ptr cur_frame);

  modules::Dataset::Ptr dataset_;
  modules::Camera::Ptr camera_;
  modules::Viewer::Ptr viewer_;

  modules::FeatureMatcher::Ptr feat_matcher_;
};
}  // namespace app
}  // namespace pose_correction

#endif  // M300_POSE_CORRECTION_INCLUDE_POSE_CORRECTION_APP_POSE_CORRECTOR_H_
