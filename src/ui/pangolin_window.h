/**
 * @file pangolin_window.h
 * @author uanheng (uanheng@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "common/types/eigen_types.h"
#include "common/types/gnss.h"
#include "common/types/nav_state.h"
#include "common/types/point_types.h"

#include <map>
#include <memory>

namespace slam_tools::ui {

class PangolinWindowImpl;

/**
 * @note 此类本身不直接涉及任何opengl和pangolin操作，应当放到PangolinWindowImpl中
 */
class PangolinWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PangolinWindow();
  ~PangolinWindow();

  /// @brief 初始化窗口，后台启动render线程。
  /// @note 与opengl/pangolin无关的初始化，尽量放到此函数体中;
  ///       opengl/pangolin相关的内容，尽量放到PangolinWindowImpl::Init中。
  bool init(const std::string& name = "slam_tools");

  /// 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
  void updatePointCloudGlobal(const std::map<Vec2i, CloudPtr, less_vec<2>>& cloud);

  /// 更新激光地图点云，在激光定位中的地图发生改变时，由fusion调用
  void updateStaticPointCloudGlobal(const CloudPtr cloud);

  /// 更新kalman滤波器状态
  void updateNavState(const NavStated& state);

  /// 更新一次scan和它对应的Pose
  void updateScan(CloudPtr cloud, const SE3& pose);

  /// 更新GPS定位结果
  void updateGps(const GNSS& gps);

  /// 等待显示线程结束，并释放资源
  void quit();

  /// 用户是否已经退出UI
  bool shouldQuit();

  /// 设置IMU到雷达的外参
  void setTImuLidar(const SE3& T_imu_lidar);

  /// 设置需要保留多少个扫描数据
  void setCurrentScanSize(int current_scan_size);

 private:
  std::shared_ptr<PangolinWindowImpl> impl_ = nullptr;
};
}  // namespace slam_tools::ui
