/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: align_ir_rgb.cpp
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
#include <opencv2/opencv.hpp>
#include "m300_depth_filter/PrintCtrlMacro.h"

bool FindFireContours(const cv::Mat binary_input,
                      std::vector<cv::Point2f>* centers,
                      std::vector<float>* radius_list,
                      std::vector<std::vector<cv::Point>>* contours,
                      const int morph_size, const bool imshow_contours,
                      const bool imshow_final_rect) {
  cv::Mat binary = binary_input.clone();

  cv::Mat show_img;
  cv::cvtColor(binary, show_img, cv::COLOR_GRAY2BGR);

  // STEP: 1 morphology
  if (morph_size > 0) {
    cv::Mat after_open_binary;
    cv::Mat after_close_binary;

    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morph_size, morph_size));

    cv::morphologyEx(binary, after_open_binary, cv::MORPH_OPEN, element);
    cv::morphologyEx(after_open_binary, after_close_binary, cv::MORPH_CLOSE,
                     element);

    binary = after_close_binary;
  }

  // STEP: 2 find contours
  cv::Mat contours_img = cv::Mat::zeros(binary.rows, binary.cols, CV_8UC3);
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary, (*contours), hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  if (contours->empty()) {
    PRINT_WARN("no contours found!");
    return false;
  }

  if (imshow_contours) {
    int index = 0;
    for (; index >= 0; index = hierarchy[index][0]) {
      cv::Scalar color(255, 255, 255);
      drawContours(contours_img, *contours, index, color, 0, 8, hierarchy);
    }

    cv::imshow("contours image:", contours_img);
    cv::waitKey(0);
  }

  // STEP: 3 find circle
  for (size_t i = 0; i < contours->size(); i++) {
    std::vector<cv::Point> points = (*contours)[i];
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(points, center, radius);
    centers->push_back(center);
    radius_list->push_back(radius);

    if (imshow_final_rect) {
      PRINT_INFO("contour size: %zu", contours->size());
      cv::circle(show_img, center, radius, cv::Scalar(0, 0, 255), 2);
      cv::circle(show_img, center, radius * 1.5, cv::Scalar(255, 0, 0), 2);
    }
  }

  if (imshow_final_rect) {
    cv::imshow("final rect", show_img);
    cv::waitKey(0);
  }

  return true;
}

int main(int argc, char** argv) {
  Eigen::Matrix3d rot;
  Eigen::Vector3d trans;

  rot << 0.80892, 0.00365467, 109.598, -0.00703363, 0.800454, 45.114,
      -3.42839e-20, 3.37542e-19, 1;
  trans << 73.0345, -19.277, 6.88341e-16;

  // translate the rgb predicted fire point to the ir images
  std::string depth_list[5] = {"17.1", "22.7", "31.6", "41.2", "51.2"};
  for (int i = 0; i < 5; i++) {
    double depth = std::stod(depth_list[i]);
    std::string ref_rgb_name =
        "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_" +
        depth_list[i] + "/rgb/0.png";
    std::string ref_mask_name =
        "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_" +
        depth_list[i] + "/mask/0.png";
    std::string ref_ir_name =
        "/home/ls/m300_depth_filter/m300_depth_data/m300_grabbed_data_1_" +
        depth_list[i] + "/ir/0.png";

    cv::Mat rgb_img = cv::imread(ref_rgb_name);
    cv::Mat mask_img = cv::imread(ref_mask_name, cv::IMREAD_GRAYSCALE);
    cv::Mat fire_mask, smoke_mask;
    cv::threshold(mask_img, fire_mask, 250, 255, cv::THRESH_BINARY);
    cv::threshold(mask_img, smoke_mask, 150, 255, cv::THRESH_TOZERO_INV);
    cv::imshow("firemask", fire_mask);
    cv::imshow("smokemask", smoke_mask);
    cv::waitKey(0);

    std::vector<float> fire_radius_list, smoke_radius_list;
    std::vector<cv::Point2f> fire_boundary_centers, smoke_boundary_centers;
    std::vector<std::vector<cv::Point>> fire_contours, smoke_contours;
    FindFireContours(fire_mask, &fire_boundary_centers, &fire_radius_list,
                     &fire_contours, 1, true, true);
    FindFireContours(smoke_mask, &smoke_boundary_centers, &smoke_radius_list,
                     &smoke_contours, 1, true, true);

    // draw the contours on the rgb image
    cv::drawContours(rgb_img, smoke_contours, -1, cv::Scalar(255, 255, 255), 2);
    cv::drawContours(rgb_img, fire_contours, -1, cv::Scalar(0, 0, 255), 2);
    cv::imshow("align", rgb_img);
    cv::waitKey(0);
    cv::imwrite("rgb_" + depth_list[i] + ".png", rgb_img);

    Eigen::Vector3d q_ir, q_rgb;

    std::vector<std::vector<cv::Point>> fire_traned_contours;
    for (auto& cont : fire_contours) {
      std::vector<cv::Point> tmp_tran_cont;
      for (auto& pt : cont) {
        q_rgb << pt.x, pt.y, 1.0;
        q_ir = rot * q_rgb + (1 / depth) * trans;
        tmp_tran_cont.push_back(
            cv::Point(q_ir[0] / q_ir[2], q_ir[1] / q_ir[2]));
      }
      fire_traned_contours.push_back(tmp_tran_cont);
    }

    std::vector<std::vector<cv::Point>> smoke_traned_contours;
    for (auto& cont : smoke_contours) {
      std::vector<cv::Point> tmp_tran_cont;
      for (auto& pt : cont) {
        q_rgb << pt.x, pt.y, 1.0;
        q_ir = rot * q_rgb + (1 / depth) * trans;
        tmp_tran_cont.push_back(
            cv::Point(q_ir[0] / q_ir[2], q_ir[1] / q_ir[2]));
      }
      smoke_traned_contours.push_back(tmp_tran_cont);
    }

    cv::Mat ir_img = cv::imread(ref_ir_name);
    cv::Mat ir_img_show = ir_img.clone();
    cv::drawContours(ir_img_show, smoke_traned_contours, -1,
                     cv::Scalar(255, 255, 255), 2);
    cv::drawContours(ir_img_show, fire_traned_contours, -1,
                     cv::Scalar(0, 0, 255), 2);
    cv::imshow("align", ir_img_show);
    cv::waitKey(0);
    cv::imwrite("ir_" + depth_list[i] + ".png", ir_img_show);
  }

  return 0;
}
