/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: Triangulator.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-06-07
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/Triangulator.hpp"
#include "m300_depth_filter/Dataset.hpp"
#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/FileWritter.hpp"
#include <opencv2/core/eigen.hpp>

namespace depth_filter {
void Triangulator::Run() {
  Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_17.1");

  // STEP: 1 find ref frame
  Frame ref_frame, cur_frame;

  if (!dataset.ReadAbsRelPose(0, &ref_frame.id_, &ref_frame.Twc_)) return;

  ref_frame.rgb_img_ =
      dataset.GetRGBImageByIndex(ref_frame.id_, cv::IMREAD_GRAYSCALE);
  ref_frame.rgb_seg_ =
      dataset.GetRGBSegImageByIndex(ref_frame.id_, cv::IMREAD_GRAYSCALE);

  if (!ref_frame.DetectFeatures()) {
    PRINT_ERROR("no features in ref frame");
    return;
  }

  // other frames
  FileWritter depth_writter("depth_estimation.csv", 9);
  depth_writter.new_open();
  depth_writter.write("index", "x", "y", "z");
  for (int line_index = 2; line_index < 300; ++line_index) {
    if (!dataset.ReadAbsRelPose(line_index, &cur_frame.id_, &cur_frame.Twc_))
      return;
    PRINT_INFO("current frame id: %d", cur_frame.id_);

    cur_frame.rgb_img_ =
        dataset.GetRGBImageByIndex(cur_frame.id_, cv::IMREAD_GRAYSCALE);
    cur_frame.rgb_seg_ =
        dataset.GetRGBSegImageByIndex(cur_frame.id_, cv::IMREAD_GRAYSCALE);

    if (!cur_frame.DetectFeatures()) {
      PRINT_WARN("no features in cur frame %d", cur_frame.id_);
      continue;
    }
    // match
    std::vector<cv::DMatch> matches;
    MacthFeaturesBF(ref_frame, cur_frame, &matches, false);

    // draw and show matches
    cv::Mat img_show;
    cv::drawMatches(ref_frame.rgb_img_, ref_frame.kps_, cur_frame.rgb_img_,
                    cur_frame.kps_, matches, img_show);
    cv::imshow("matched", img_show);
    cv::waitKey(1);

    if (matches.empty()) {
      PRINT_WARN("No matches in cur frame!");
      continue;
    }

    // triangulation
    std::vector<cv::Point2f> matched_ref_pts, matched_cur_pts;
    for (auto m : matches) {
      matched_ref_pts.push_back(ref_frame.kps_[m.queryIdx].pt);
      matched_cur_pts.push_back(cur_frame.kps_[m.trainIdx].pt);
    }

    std::vector<Eigen::Vector3d> fire_points;

    Triangulation(ref_frame.Twc_.inverse(), cur_frame.Twc_.inverse(),
                  matched_ref_pts, matched_cur_pts, &fire_points);

    PRINT_INFO("the 3d points in ref coordinates total: %zu",
               fire_points.size());
    for (size_t no = 0; no < fire_points.size(); ++no) {
      std::cout << "fire point: " << no << ": " << fire_points[no].transpose()
                << std::endl;
      depth_writter.write(cur_frame.id_, fire_points[no](0), fire_points[no](1),
                          fire_points[no](2));
    }
  }
}

void Triangulator::MacthFeaturesBF(const Frame& ref, const Frame& cur,
                                   std::vector<cv::DMatch>* matches,
                                   const bool use_hist) {
  std::vector<cv::DMatch> raw_matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(ref.descriptor_, cur.descriptor_, raw_matches);
  std::vector<cv::DMatch> good_matches, refined_hist_matches;

  // distance
  for (cv::DMatch& match : raw_matches) {
    if (match.distance <= 40) good_matches.push_back(match);
  }

  if (use_hist) {
    CullWithHistConsistency(ref.kps_, cur.kps_, good_matches,
                            refined_hist_matches);
    (*matches) = refined_hist_matches;
  } else {
    (*matches) = good_matches;
  }
}

void Triangulator::CullWithHistConsistency(
    const std::vector<cv::KeyPoint>& keypoints_ref,
    const std::vector<cv::KeyPoint>& keypoints_cur,
    const std::vector<cv::DMatch>& raw_matches,
    std::vector<cv::DMatch>& good_matches) {
  const int HISTO_LENGTH = 30;
  std::vector<cv::DMatch> rot_hist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; ++i) {
    rot_hist[i].reserve(500);
  }
  const float factor = HISTO_LENGTH / 360.0f;

  for (cv::DMatch match : raw_matches) {
    float angle = keypoints_ref[match.queryIdx].angle -
                  keypoints_cur[match.trainIdx].angle;
    if (angle < 0.0f) {
      angle += 360.f;
    }
    int bin = std::round(angle * factor);
    if (bin == HISTO_LENGTH) {
      bin = 0;
    }

    rot_hist[bin].push_back(match);
  }

  // find the most number in histogram
  size_t max_index = 0;
  size_t max_num = 0;
  for (int i = 0; i < HISTO_LENGTH; ++i) {
    if (rot_hist[i].size() > max_num) {
      max_num = rot_hist[i].size();
      max_index = i;
    }
  }

  good_matches = rot_hist[max_index];
}

void Triangulator::Triangulation(const Sophus::SE3d T_ref,
                                 const Sophus::SE3d T_cur,
                                 const std::vector<cv::Point2f>& pts_ref,
                                 const std::vector<cv::Point2f>& pts_cur,
                                 std::vector<Eigen::Vector3d>* points_3d) {
  cv::Mat T1, T2;
  cv::eigen2cv(T_ref.matrix3x4(), T1);
  cv::eigen2cv(T_cur.matrix3x4(), T2);

  cv::Mat K =
      (cv::Mat_<double>(3, 3) << 2075.220169334596, 0, 456.1915029618301, 0,
       2074.731865204465, 402.94925809268, 0, 0, 1);

  cv::Mat P1 = K * T1, P2 = K * T2;
  cv::Mat final_points;
  cv::triangulatePoints(P1, P2, pts_ref, pts_cur, final_points);

  for (int i = 0; i < final_points.cols; i++) {
    cv::Mat x = final_points.col(i);
    x /= x.at<float>(3, 0);

    Eigen::Vector3d p;
    p << x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0);

    points_3d->push_back(p);
  }
}

}  // namespace depth_filter

int main(int argc, char** argv) {
  depth_filter::Triangulator tri;
  tri.Run();

  return 0;
}
