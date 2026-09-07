/**
 * @file ui_cloud.h
 * @author uanheng (uanheng@foxmail.com)
 * @brief 点云显示绘制
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "common/types/eigen_types.h"
#include "common/types/point_types.h"

#include <pangolin/gl/glvbo.h>

namespace slam_tools::ui {

/// 在UI中使用的点云
/// 固定不变的点云都可以用这个来渲染
class UiCloud {
 public:
  /// 使用哪种颜色渲染本点云
  enum UseColor {
    PCL_COLOR,        // PCL颜色，偏红
    INTENSITY_COLOR,  // 亮度
    HEIGHT_COLOR,     // 高度
    GRAY_COLOR,       // 显示为灰色
  };

  UiCloud() {}
  explicit UiCloud(CloudPtr cloud);

  /**
   * 从PCL点云来设置一个UI点云
   * @param cloud             PCL 点云
   * @param pose              点云位姿，设置之后转换到全局坐标系
   */
  void setCloud(CloudPtr cloud, const SE3& pose);

  /// 渲染这个点云
  void render();

  void setRenderColor(UseColor use_color);

 private:
  Vec4f intensityToRgbPCL(const float& intensity) const {
    int index = static_cast<int>(intensity * 6);
    index = static_cast<int>(index % intensity_color_table_pcl_.size());
    return intensity_color_table_pcl_[index];
  }

  UseColor use_color_ = UseColor::PCL_COLOR;

  std::vector<Vec3f> xyz_data_;              // XYZ buffer
  std::vector<Vec4f> color_data_pcl_;        // color buffer
  std::vector<Vec4f> color_data_intensity_;  // color buffer
  std::vector<Vec4f> color_data_height_;     // color buffer
  std::vector<Vec4f> color_data_gray_;       // color buffer

  pangolin::GlBuffer vbo_;  // 显存顶点信息
  pangolin::GlBuffer cbo_;  // 颜色顶点信息

  /// PCL中intensity table
  void buildIntensityTable();
  static std::vector<Vec4f> intensity_color_table_pcl_;
};

}  // namespace slam_tools::ui