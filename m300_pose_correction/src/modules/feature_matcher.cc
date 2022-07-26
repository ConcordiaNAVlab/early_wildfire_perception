/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: feature_matcher.cc
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

#include "pose_correction/modules/feature_matcher.h"

namespace pose_correction {
namespace modules {

void FeatureMatcher::TrackFeaturesLK(const modules::Frame::Ptr ref_frame,
                                     const modules::Frame::Ptr cur_frame,
                                     std::vector<cv::Point2f>& tracked_pts_ref,
                                     std::vector<cv::Point2f>& tracked_pts_cur,
                                     bool show_trackes) {
  std::vector<cv::Point2f> pts_ref = ref_frame->kps_pt_,
                           pts_cur = cur_frame->kps_pt_;

  std::vector<float> error;
  std::vector<uchar> status;
  cv::Size win_size = cv::Size(21, 21);
  cv::TermCriteria term_crit = cv::TermCriteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

  cv::calcOpticalFlowPyrLK(ref_frame->img_, cur_frame->img_, pts_ref, pts_cur,
                           status, error, win_size, 3, term_crit);

  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i] == 1) {
      tracked_pts_ref.push_back(pts_ref[i]);
      tracked_pts_cur.push_back(pts_cur[i]);
    }
  }

  if (show_trackes) {
    cv::Mat img_to_draw;
    cv::cvtColor(cur_frame->img_.clone(), img_to_draw, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < tracked_pts_cur.size(); ++i) {
      cv::circle(img_to_draw, tracked_pts_cur[i], 2, cv::Scalar(0, 250, 0), 2);
      cv::line(img_to_draw, tracked_pts_cur[i], tracked_pts_ref[i],
               cv::Scalar(0, 250, 0), 2);
    }
    cv::imshow("current image", img_to_draw);
    cv::waitKey(0);
  }
}

void FeatureMatcher::MacthFeaturesBF(const modules::Frame::Ptr ref_frame,
                                     const modules::Frame::Ptr cur_frame,
                                     std::vector<cv::Point2f>& match_pts_ref,
                                     std::vector<cv::Point2f>& match_pts_cur,
                                     bool show_matches) {
  std::vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(ref_frame->descriptors_, cur_frame->descriptors_, matches);

  std::vector<cv::DMatch> good_matches, refined_hist_matches;

  // distance
  for (cv::DMatch& match : matches) {
    if (match.distance <= 20) good_matches.push_back(match);
  }

  CullWithHistConsistency(ref_frame->kps_, cur_frame->kps_, good_matches,
                          refined_hist_matches);

  // fill all the refined matches (need to check)
  for (cv::DMatch& m : refined_hist_matches) {
    int ref_index = m.queryIdx;
    int cur_index = m.trainIdx;

    match_pts_ref.push_back(ref_frame->kps_pt_[ref_index]);
    match_pts_cur.push_back(cur_frame->kps_pt_[cur_index]);
  }

  if (show_matches) {
    cv::Mat img_good_match;
    cv::Mat ref_show, cur_show;
    cv::cvtColor(ref_frame->img_, ref_show, cv::COLOR_GRAY2BGR);
    cv::cvtColor(cur_frame->img_, cur_show, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < match_pts_ref.size(); ++i) {
      cv::circle(ref_show, match_pts_ref[i], 5, cv::Scalar(0, 250, 0), 2);
      cv::circle(cur_show, match_pts_cur[i], 5, cv::Scalar(0, 250, 0), 2);
    }

    cv::drawMatches(ref_show, ref_frame->kps_, cur_show, cur_frame->kps_,
                    refined_hist_matches, img_good_match);

    cv::imshow("matches", img_good_match);
    cv::waitKey(0);
  }
}

void FeatureMatcher::CullWithHistConsistency(
    const std::vector<cv::KeyPoint>& keypoints_ref,
    const std::vector<cv::KeyPoint>& keypoints_cur,
    const std::vector<cv::DMatch>& raw_matches,
    std::vector<cv::DMatch>& good_matches) {
  const int HISTO_LENGTH = 60;
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
}  // namespace modules
}  // namespace pose_correction
