/**
 * @file ui_trajectory.cc
 * @author uanheng (uanheng@foxmail.com)
 * @brief 轨迹绘制
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ui_trajectory.h"

namespace slam_tools::ui {

void UiTrajectory::addPt(const SE3& pose) {
  pos_.emplace_back(pose.translation().cast<float>());
  if (pos_.size() > max_size_) {
    pos_.erase(pos_.begin(), pos_.begin() + static_cast<int64_t>(pos_.size()) / 2);  // 删掉前一半的点
  }
  vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, pos_);
}

void UiTrajectory::render() {
  if (!vbo_.IsValid()) {
    return;
  }

  glColor3f(color_[0], color_[1], color_[2]);

  // 点线形式
  glLineWidth(3.0);
  pangolin::RenderVbo(vbo_, GL_LINE_STRIP);
  glLineWidth(1.0);

  glPointSize(5.0);
  pangolin::RenderVbo(vbo_, GL_POINTS);
  glPointSize(1.0);
}

}  // namespace slam_tools::ui