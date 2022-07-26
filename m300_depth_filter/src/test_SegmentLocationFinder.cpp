/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: test_SegmentLocationFinder.cpp
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-05-16
 *
 *   @Description:
 *
 *******************************************************************************/

#include "m300_depth_filter/PrintCtrlMacro.h"
#include "m300_depth_filter/SegmentLocationFinder.hpp"
#include "m300_depth_filter/Dataset.hpp"

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <sophus/se3.hpp>

const float patch_width = 90.0f;
const float patch_height = 90.0f;

int main(int argc, char** argv) {
  std::vector<std::string> rgb_img_names, mask_img_names;
  depth_filter::Dataset dataset(
      "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_51.2");
  dataset.GetAllRGBImageNames(&rgb_img_names);
  dataset.GetAllRGBSegImageNames(&mask_img_names);
  depth_filter::SegmentLocationFinder finder;

  // for each of the images
  cv::Mat ref_img, ref_desc;
  std::vector<cv::KeyPoint> ref_kps;

  for (size_t i = 0; i < rgb_img_names.size(); ++i) {
    PRINT_DEBUG("Frame ID: %zu", i);
    cv::Mat mask_img = cv::imread(mask_img_names[i], cv::IMREAD_GRAYSCALE);
    cv::Mat mask_binary;
    cv::threshold(mask_img, mask_binary, 250, 255, cv::THRESH_BINARY);

    std::vector<float> radius_list;
    std::vector<cv::Point2f> boundary_centers;
    if (!finder.FindLocation(mask_binary, &boundary_centers, &radius_list, 5,
                             true, true))
      continue;

    cv::Mat rgb_img = cv::imread(rgb_img_names[i], cv::IMREAD_GRAYSCALE);
    cv::Mat rgb_show1 = rgb_img.clone();

    // STEP: 在patch 中提取特征点

    cv::Mat patch_mask = cv::Mat::zeros(rgb_img.size(), CV_8UC1);
      cv::Mat img_contour;
      cv::cvtColor(rgb_img, img_contour, cv::COLOR_GRAY2BGR);
    for (size_t i = 0; i < boundary_centers.size(); ++i) {
      cv::circle(patch_mask, boundary_centers[i], radius_list[i] * 1.5,
                 cv::Scalar(255, 255, 255), -1);

      cv::circle(img_contour, boundary_centers[i], radius_list[i],
                 cv::Scalar(0, 0, 255), 2);
      cv::circle(img_contour, boundary_centers[i], radius_list[i] * 1.5,
                 cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow("patch", patch_mask);

    // 提取orb角点
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    std::vector<cv::KeyPoint> kps;
    detector->detect(rgb_img, kps, patch_mask);
    cv::Mat rgb_with_kps = rgb_img.clone();
    cv::drawKeypoints(rgb_img, kps, rgb_with_kps);
    cv::imshow("kps", rgb_with_kps);

    cv::Mat descriptors;
    detector->compute(rgb_img, kps, descriptors);

    if (i == 23) {
      ref_img = rgb_img;
      ref_desc = descriptors;
      ref_kps = kps;
      cv::imwrite("contour_rgb.png", img_contour);
    }

    if (i >= 23) {
      // match
      std::vector<cv::DMatch> matches;
      cv::BFMatcher matcher(cv::NORM_HAMMING);
      matcher.match(ref_desc, descriptors, matches);
      std::vector<cv::DMatch> good_matches;

      // distance
      for (cv::DMatch& match : matches) {
        if (match.distance <= 40) good_matches.push_back(match);
      }

      cv::Mat img_good_match;
      cv::drawMatches(ref_img, ref_kps, rgb_img, kps, good_matches,
                      img_good_match);
      cv::imwrite("matches.png", img_good_match);

      cv::imshow("matches", img_good_match);
    }

    cv::waitKey(0);
  }

  return 0;
}
