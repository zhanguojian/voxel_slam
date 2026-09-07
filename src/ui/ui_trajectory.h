/**
 * @file ui_trajectory.h
 * @author uanheng (uanheng@foxmail.com)
 * @brief 轨迹绘制
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "common/types/eigen_types.h"

#include <pangolin/gl/glvbo.h>

namespace slam_tools::ui {

/// UI中的轨迹绘制
class UiTrajectory {
 public:
  explicit UiTrajectory(const Vec3f& color) : color_(color) { pos_.reserve(max_size_); }

  /// 增加一个轨迹点
  void addPt(const SE3& pose);

  /// 渲染此轨迹
  void render();

  void clear() {
    pos_.clear();
    pos_.reserve(max_size_);
    vbo_.Free();
  }

 private:
  uint32_t max_size_ = 1e6;      // 记录的最大点数
  std::vector<Vec3f> pos_;       // 轨迹记录数据
  Vec3f color_ = Vec3f::Zero();  // 轨迹颜色显示
  pangolin::GlBuffer vbo_;       // 显存顶点信息
};

}  // namespace slam_tools::ui
