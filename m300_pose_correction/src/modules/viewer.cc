/*******************************************************************************
 *   Copyright (C) 2022 Concordia NAVlab. All rights reserved.
 *
 *   @Filename: viewer.cc
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

#include "pose_correction/modules/viewer.h"
#include <unistd.h>

namespace pose_correction {
namespace modules {

Viewer::Viewer() {
  viewer_running_.store(true);
  viewer_thread_ = std::thread(std::bind(&Viewer::DrawTrajectory, this));
}

void Viewer::Stop() {
  viewer_running_.store(false);
  printf("stop current viewer! wait for join!");
  viewer_thread_.join();
}

void Viewer::Update(const TrajType traj) {
  // NOTE: 更新traj_时上锁
  std::unique_lock<std::mutex> lck(traj_mutex_);
  traj_ = traj;
}

void Viewer::DrawTrajectory() {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false && viewer_running_.load()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    // 画出连线
    for (size_t i = 0; i + 1 < traj_.size(); i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = traj_[i], p2 = traj_[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);  // sleep 5 ms
  }
}
}  // namespace modules
}  // namespace pose_correction
