/**
 * @file pangolin_window_impl.cc
 * @author uanheng (uanheng@foxmail.com)
 * @brief
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <glog/logging.h>
#include <string>

#include <pangolin/display/default_font.h>
#include "tools/ui/pangolin_window_impl.h"

namespace slam_tools::ui {

using UL = std::unique_lock<std::mutex>;

bool PangolinWindowImpl::init(const std::string &win_name) {
  win_name_ = win_name;

  // create a window and bind its context to the main thread
  pangolin::CreateWindowAndBind(win_name, win_width_, win_height_);

  // 3D mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // opengl buffer
  allocateBuffer();

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();

  // 轨迹
  traj_lidarloc_ui_.reset(new ui::UiTrajectory(Vec3f(1.0, 0.0, 0.0)));      // 红色
  traj_gps_ui_.reset(new ui::UiTrajectory(Vec3f(1.0, 1.0, 51.0 / 255.0)));  // 黄色

  current_scan_.reset(new PointCloudType);
  current_scan_ui_.reset(new ui::UiCloud);

  /// data log
  log_vel_.SetLabels(std::vector<std::string>{"vel_x", "vel_y", "vel_z"});
  log_vel_baselink_.SetLabels(std::vector<std::string>{"baselink_vel_x", "baselink_vel_y", "baselink_vel_z"});
  log_bias_acc_.SetLabels(std::vector<std::string>{"ba_x", "ba_y", "ba_z"});
  log_bias_gyr_.SetLabels(std::vector<std::string>{"bg_x", "bg_y", "bg_z"});

  return true;
}

bool PangolinWindowImpl::deInit() {
  releaseBuffer();
  return true;
}

bool PangolinWindowImpl::updateGlobalMap() {
  if (!cloud_global_need_update_.load()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_map_cloud_);
  for (const auto &cp : cloud_global_map_) {
    if (cloud_map_ui_.find(cp.first) != cloud_map_ui_.end()) {
      continue;
    }

    std::shared_ptr<ui::UiCloud> ui_cloud(new ui::UiCloud);
    ui_cloud->setCloud(cp.second, SE3());
    ui_cloud->setRenderColor(ui::UiCloud::UseColor::GRAY_COLOR);
    cloud_map_ui_.emplace(cp.first, ui_cloud);
  }

  for (auto iter = cloud_map_ui_.begin(); iter != cloud_map_ui_.end();) {
    if (cloud_global_map_.find(iter->first) == cloud_global_map_.end()) {
      iter = cloud_map_ui_.erase(iter);
    } else {
      iter++;
    }
  }
  cloud_global_need_update_.store(false);

  return true;
}

bool PangolinWindowImpl::updateGlobalStaticMap() {
  if (!cloud_static_global_need_update_.load()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_map_cloud_);
  std::shared_ptr<ui::UiCloud> ui_cloud(new ui::UiCloud);
  ui_cloud->setCloud(cloud_static_global_map_, SE3());
  ui_cloud->setRenderColor(ui::UiCloud::UseColor::GRAY_COLOR);
  cloud_map_ui_.clear();
  cloud_map_ui_.emplace(Vec2i(0, 0), ui_cloud);

  cloud_static_global_need_update_.store(false);

  return true;
}

bool PangolinWindowImpl::updateCurrentScan() {
  UL lock(mtx_current_scan_);
  if (current_scan_ != nullptr && !current_scan_->empty() && current_scan_need_update_) {
    if (current_scan_ui_) {
      current_scan_ui_->setRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);
      scans_.emplace_back(current_scan_ui_);
    }

    current_scan_ui_ = std::make_shared<ui::UiCloud>();
    current_scan_ui_->setCloud(current_scan_, current_pose_);
    current_scan_ui_->setRenderColor(ui::UiCloud::UseColor::HEIGHT_COLOR);

    current_scan_need_update_.store(false);
  }

  if (scans_.size() > max_size_of_current_scan_) {
    scans_.pop_front();
  }

  return true;
}

bool PangolinWindowImpl::updateState() {
  if (!kf_result_need_update_.load()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_nav_state_);
  Vec3d vel_baselink = pose_.so3().inverse() * vel_;

  // 滤波器状态作曲线图
  log_vel_.Log(static_cast<float>(vel_(0)), static_cast<float>(vel_(1)), static_cast<float>(vel_(2)));
  log_vel_baselink_.Log(static_cast<float>(vel_baselink(0)), static_cast<float>(vel_baselink(1)),
                        static_cast<float>(vel_baselink(2)));
  log_bias_acc_.Log(static_cast<float>(bias_acc_(0)), static_cast<float>(bias_acc_(1)),
                    static_cast<float>(bias_acc_(2)));
  log_bias_gyr_.Log(static_cast<float>(bias_gyr_(0)), static_cast<float>(bias_gyr_(1)),
                    static_cast<float>(bias_gyr_(2)));

  current_pose_ = pose_;
  traj_lidarloc_ui_->addPt(current_pose_);

  kf_result_need_update_.store(false);
  return false;
}

bool PangolinWindowImpl::updateGps() {
  if (!gps_need_update_.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mtx_gps_pose_);

  // 更新定位轨迹
  traj_gps_ui_->addPt(gps_pose_);
  gps_need_update_.store(false);
  return true;
}

void PangolinWindowImpl::drawAll() {
  for (const auto &pc : cloud_map_ui_) {
    pc.second->render();
  }

  for (const auto &s : scans_) {
    s->render();
  }

  current_scan_ui_->render();

  traj_lidarloc_ui_->render();
  traj_gps_ui_->render();

  // 车
  car_.setPose(current_pose_);  // 车在current pose上
  car_.render();

  // 文字
  renderLabels();
}

void PangolinWindowImpl::renderClouds() {
  // 更新各种推送过来的状态
  updateGlobalMap();
  updateGlobalStaticMap();
  updateState();
  updateGps();
  updateCurrentScan();

  // 绘制
  pangolin::Display(dis_3d_main_name_).Activate(s_cam_main_);
  drawAll();
}

void PangolinWindowImpl::renderLabels() {
  // 定位状态标识，显示在3D窗口中
  auto &d_cam3d_main = pangolin::Display(dis_3d_main_name_);
  d_cam3d_main.Activate(s_cam_main_);
  const auto cur_width = d_cam3d_main.v.w;
  const auto cur_height = d_cam3d_main.v.h;

  GLint view[4];
  glGetIntegerv(GL_VIEWPORT, view);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glOrtho(0, cur_width, 0, cur_height, -1, 1);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glTranslatef(5, static_cast<float>(cur_height - 1.5 * gltext_label_global_.Height()), 1.0);
  glColor3ub(127, 127, 127);
  gltext_label_global_.Draw();

  // Restore modelview / project matrices
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void PangolinWindowImpl::createDisplayLayout() {
  // define camera render object (for view / scene browsing)
  auto proj_mat_main = pangolin::ProjectionMatrix(win_width_, win_width_, cam_focus_, cam_focus_, win_width_ / 2,
                                                  win_width_ / 2, cam_z_near_, cam_z_far_);
  auto model_view_main = pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, pangolin::AxisY);
  s_cam_main_ = pangolin::OpenGlRenderState(std::move(proj_mat_main), std::move(model_view_main));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam3d_main = pangolin::Display(dis_3d_main_name_)
                                     .SetBounds(0.0, 1.0, 0.0, 1.0)
                                     .SetHandler(new pangolin::Handler3D(s_cam_main_));

  pangolin::View &d_cam3d = pangolin::Display(dis_3d_name_)
                                .SetBounds(0.0, 1.0, 0.0, 0.75)
                                .SetLayout(pangolin::LayoutOverlay)
                                .AddDisplay(d_cam3d_main);

  // OpenGL 'view' of data. We might have many views of the same data.
  plotter_vel_ = std::make_unique<pangolin::Plotter>(&log_vel_, -10, 600, -11, 11, 75, 2);
  plotter_vel_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_vel_->Track("$i");
  plotter_vel_->SetBackgroundColour(pangolin::Colour(248. / 255., 248. / 255., 255. / 255.));
  plotter_vel_baselink_ = std::make_unique<pangolin::Plotter>(&log_vel_baselink_, -10, 600, -11, 11, 75, 2);
  plotter_vel_baselink_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_vel_baselink_->Track("$i");
  plotter_vel_baselink_->SetBackgroundColour(pangolin::Colour(1.0, 1.0, 240 / 255.0));
  plotter_bias_acc_ = std::make_unique<pangolin::Plotter>(&log_bias_acc_, -10, 600, -2.0, 2.0, 75, 0.01);
  plotter_bias_acc_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_bias_acc_->Track("$i");
  plotter_bias_acc_->SetBackgroundColour(pangolin::Colour(255.0 / 255.0, 240.0 / 255.0, 245.0 / 255.0));
  plotter_bias_gyr_ = std::make_unique<pangolin::Plotter>(&log_bias_gyr_, -10, 600, -0.1, 0.1, 75, 0.01);
  plotter_bias_gyr_->SetBounds(0.02, 0.98, 0.0, 1.0);
  plotter_bias_gyr_->Track("$i");
  plotter_bias_gyr_->SetBackgroundColour(pangolin::Colour(224.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0));

  pangolin::View &d_plot = pangolin::Display(dis_plot_name_)
                               .SetBounds(0.0, 1.0, 0.75, 1.0)
                               .SetLayout(pangolin::LayoutEqualVertical)
                               .AddDisplay(*plotter_bias_acc_)
                               .AddDisplay(*plotter_bias_gyr_)
                               .AddDisplay(*plotter_vel_)
                               .AddDisplay(*plotter_vel_baselink_);

  pangolin::Display(dis_main_name_)
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(menu_width_), 1.0)
      .AddDisplay(d_cam3d)
      .AddDisplay(d_plot);
}

void PangolinWindowImpl::render() {
  // fetch the context and bind it to this thread
  pangolin::BindToContext(win_name_);

  // Issue specific OpenGl we might need
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // menu
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(menu_width_));
  pangolin::Var<bool> menu_follow_loc("menu.Follow", false, true);
  pangolin::Var<bool> menu_reset_3d_view("menu.Reset 3D View", false, false);

  // display layout
  createDisplayLayout();

  exit_flag_.store(false);
  while (!pangolin::ShouldQuit() && !exit_flag_) {
    // clear entire screen
    glClearColor(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // menu control
    following_loc_ = menu_follow_loc;

    if (menu_reset_3d_view) {
      s_cam_main_.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, pangolin::AxisY));
      menu_reset_3d_view = false;
    }

    // render pointcloud
    renderClouds();

    /// 处理相机跟随问题
    if (following_loc_) {
      s_cam_main_.Follow(current_pose_.matrix());
    }

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  // unset the current context from the main thread
  pangolin::GetBoundWindow()->RemoveCurrent();
}

void PangolinWindowImpl::allocateBuffer() {
  std::string global_text("Welcome to slam_tools.UI. ");
  auto &font = pangolin::default_font();
  gltext_label_global_ = font.Text(global_text);
}

void PangolinWindowImpl::releaseBuffer() {}

}  // namespace slam_tools::ui
