
#pragma once

#include <pangolin/gl/glvbo.h>

#include "msg_types.h"

/// 在UI里显示的小车
class UiCar {
 public:
  UiCar(const Vec3f& color) : color_(color) {}

  /// 设置小车 Pose，重设显存中的点
  void setPose(const SE3& pose);

  /// 渲染小车
  void render();

 private:
  Vec3f color_;
  pangolin::GlBuffer vbo_;  // buffer data

  static std::vector<Vec3f> car_vertices_;  // 小车的顶点
};

