/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: align_ir_rgb_g2o.h
 *
 *   @Author: Shun Li
 *
 *   @Email: 2015097272@qq.com
 *
 *   @Date: 2022-04-13
 *
 *   @Description:
 *
 *******************************************************************************/

#ifndef INCLUDE_ALIGN_IR_RGB_ALIGN_IR_RGB_G2O_H_
#define INCLUDE_ALIGN_IR_RGB_ALIGN_IR_RGB_G2O_H_

#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

struct RotationAndTrans {
  RotationAndTrans() {}
  explicit RotationAndTrans(double* data_addr) {
    rotation << data_addr[0], data_addr[1], data_addr[2], data_addr[3],
        data_addr[4], data_addr[5], data_addr[6], data_addr[7], data_addr[8];

    translation = Eigen::Vector3d(data_addr[9], data_addr[10], data_addr[11]);
  }

  // data
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

class RoteTransVertex : public g2o::BaseVertex<12, RotationAndTrans> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  RoteTransVertex() {}

  void setToOriginImpl() override { _estimate = RotationAndTrans(); }

  void oplusImpl(const double* update) override {
    Eigen::Matrix3d update_rotation;
    update_rotation << update[0], update[1], update[2], update[3], update[4],
        update[5], update[6], update[7], update[8];

    _estimate.rotation += update_rotation;
    _estimate.translation += Eigen::Vector3d(update[9], update[10], update[11]);
  }

  bool read(std::istream& in) override { return true; }

  bool write(std::ostream& out) const override { return true; }
};

class AlignErrEdge
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, RoteTransVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  AlignErrEdge(Eigen::Vector3d rgb, double z_in): pixel_rgb(rgb), z(z_in) {
  }

  void computeError() override {
    RoteTransVertex* rote_trans_vertex =
        static_cast<RoteTransVertex*>(_vertices[0]);

    Eigen::Vector3d ir_esta =
        rote_trans_vertex->estimate().rotation * pixel_rgb+
        1 / z * rote_trans_vertex->estimate().translation;

    _error =  _measurement - ir_esta;
  }

  bool read(std::istream& in) override { return true; }

  bool write(std::ostream& out) const override { return true; }

 private:
  Eigen::Vector3d pixel_rgb;
  double z;
};

#endif  // INCLUDE_ALIGN_IR_RGB_ALIGN_IR_RGB_G2O_H_
