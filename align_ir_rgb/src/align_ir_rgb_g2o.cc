/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: align_ir_rgb_g2o.cc
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

#include "align_ir_rgb/align_ir_rgb_g2o.h"
#include "align_ir_rgb/tools.h"
#include "align_ir_rgb/PrintControl/FileWritter.hpp"

#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

int main(int argc, char** argv) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<12, 1>> BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(
          g2o::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  // to be estimated r1-r9, t1-t3
  double r_t[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  RotationAndTrans init_rot_trans = RotationAndTrans(r_t);

  // vertex
  RoteTransVertex* v = new RoteTransVertex;
  v->setEstimate(init_rot_trans);
  v->setId(0);
  optimizer.addVertex(v);

  // edges
  std::vector<std::string> measurement_folders =
      GetSubFolders("/home/ls/m300_rgb_ir_registration/align_images");

  int edge_index = 0;
  std::vector<AlignErrEdge*> all_edges;
  for (std::string path_iter : measurement_folders) {
    EachMeasurement::Ptr measure = EachMeasurement::CreateFromFolder(path_iter);
    if (nullptr != measure) {
      for (int i = 0; i < 4; ++i) {
        AlignErrEdge* edge = new AlignErrEdge(
            measure->homo_rgb_pixel_pos_.at(i), measure->distance_);
        edge->setId(edge_index);
        edge->setVertex(0, v);
        edge->setMeasurement(measure->homo_ir_pixel_pos_.at(i));
        edge->setInformation(Eigen::Matrix3d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber);
        optimizer.addEdge(edge);
        all_edges.push_back(edge);
        ++edge_index;
      }
    }
  }

  std::cout << "start optimization" << std::endl;
  optimizer.initializeOptimization();
  optimizer.optimize(30);

  const double chi2_th = 5.991;
  int cnt_outlier = 0;
  for (int iteration = 0; iteration < 4; ++iteration) {
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    for (auto e : all_edges) {
      e->computeError();
      if (e->chi2() > chi2_th) {
        e->setLevel(1);
        ++cnt_outlier;
      }

      if (iteration == 2) {
        e->setRobustKernel(nullptr);
      }
    }
  }

  std::cout << "outliers number: " << cnt_outlier << std::endl;

  // 输出优化值
  RotationAndTrans r_t_estimate = v->estimate();
  std::cout << "final rote: \n" << r_t_estimate.rotation << std::endl;
  std::cout << "final trans: \n"
            << r_t_estimate.translation.transpose() << std::endl;

  // 检验
  tools::FileWritter error_writer("error.csv", 8);
  tools::FileWritter mean_sq_error_writer("mean_sq_error.csv", 8);
  error_writer.new_open();
  mean_sq_error_writer.new_open();

  error_writer.write("distance", "x", "y");
  mean_sq_error_writer.write("distance", "mean_sq");

  for (std::string path_iter : measurement_folders) {
    EachMeasurement::Ptr measure = EachMeasurement::CreateFromFolder(path_iter);

    if (nullptr != measure) {
      double sum_dis = 0.0;
      for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d res =
            measure->homo_ir_pixel_pos_.at(i) -
            r_t_estimate.rotation * measure->homo_rgb_pixel_pos_.at(i) +
            1 / measure->distance_ * r_t_estimate.translation;

        std::cout << "distance:" << measure->distance_
                  << "res: " << res.transpose() << std::endl;
        error_writer.write(measure->distance_, fabs(res[0]), fabs(res[1]));

        sum_dis += sqrt(res[0] * res[0] + res[1] * res[1]);
      }
      mean_sq_error_writer.write(measure->distance_, 0.25 * sum_dis);
    }
  }

  return 0;
}
