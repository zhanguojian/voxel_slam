/**
 * @file ui_car.cc
 * @author uanheng (uanheng@foxmail.com)
 * @brief 坐标轴绘制
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "ui_car.h"

namespace slam_tools::ui {

std::vector<Vec3f> UiCar::car_vertices_ = {
    // clang-format off
     { 0, 0, 0}, { 5, 0, 0},
     { 0, 0, 0}, { 0, 3, 0},
     { 0, 0, 0}, { 0, 0, 1},
    // clang-format on
};

void UiCar::setPose(const SE3& pose) {
  std::vector<Vec3f> pts;
  for (auto& p : car_vertices_) {
    pts.emplace_back(p);
  }

  // 转换到世界系
  auto pose_f = pose.cast<float>();
  for (auto& pt : pts) {
    pt = pose_f * pt;
  }

  /// 上传到显存
  vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, pts);
}

void UiCar::render() {
  if (vbo_.IsValid()) {
    glColor3f(color_[0], color_[1], color_[2]);
    pangolin::RenderVbo(vbo_, GL_LINES);
  }
}

}  // namespace slam_tools::ui