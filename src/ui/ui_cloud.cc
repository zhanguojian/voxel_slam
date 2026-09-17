
#include "ui_cloud.h"

#include <execution>
#include <numeric>

std::vector<Vec4f> UiCloud::intensity_color_table_pcl_;

UiCloud::UiCloud(CloudPtr cloud) { setCloud(cloud, SE3()); }

void UiCloud::setCloud(CloudPtr cloud, const SE3& pose) {
  if (intensity_color_table_pcl_.empty()) {
    buildIntensityTable();
  }

  assert(cloud != nullptr && cloud->empty() == false);
  xyz_data_.resize(cloud->size());
  color_data_pcl_.resize(cloud->size());
  color_data_intensity_.resize(cloud->size());
  color_data_height_.resize(cloud->size());
  color_data_gray_.resize(cloud->size());

  std::vector<int> idx(cloud->size());
  std::iota(idx.begin(), idx.end(), 0);

  SE3f pose_f = pose.cast<float>();
  std::for_each(std::execution::par_unseq, idx.begin(), idx.end(), [&](const int& id) {
    const auto& pt = cloud->points[id];
    auto pt_world = pose_f * cloud->points[id].getVector3fMap();
    xyz_data_[id] = Vec3f(pt_world.x(), pt_world.y(), pt_world.z());
    color_data_pcl_[id] = intensityToRgbPCL(pt.intensity);
    color_data_gray_[id] = Vec4f(0.5, 0.5, 0.5, 0.2);
    color_data_height_[id] = intensityToRgbPCL(pt.z * 10);
    color_data_intensity_[id] =
        Vec4f(static_cast<float>(pt.intensity / 255.0 * 3.0), static_cast<float>(pt.intensity / 255.0 * 3.0),
              static_cast<float>(pt.intensity / 255.0 * 3.0), 0.2);
  });

  vbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, xyz_data_);
}

void UiCloud::render() {
  if (vbo_.IsValid() && cbo_.IsValid()) {
    pangolin::RenderVboCbo(vbo_, cbo_);
  }
}

void UiCloud::buildIntensityTable() {
  intensity_color_table_pcl_.reserve(255 * 6);
  auto make_color = [](int r, int g, int b) -> Vec4f {
    return Vec4f(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f, 0.2f);
  };
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(255, i, 0));
  }
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(255 - i, 0, 255));
  }
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(0, 255, i));
  }
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(255, 255 - i, 0));
  }
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(i, 0, 255));
  }
  for (int i = 0; i < 256; i++) {
    intensity_color_table_pcl_.emplace_back(make_color(0, 255, 255 - i));
  }
}

void UiCloud::setRenderColor(UiCloud::UseColor use_color) {
  use_color_ = use_color;

  if (use_color_ == UseColor::PCL_COLOR) {
    cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_pcl_);
  } else if (use_color_ == UseColor::INTENSITY_COLOR) {
    cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_intensity_);
  } else if (use_color_ == UseColor::HEIGHT_COLOR) {
    cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_height_);
  } else if (use_color_ == UseColor::GRAY_COLOR) {
    cbo_ = pangolin::GlBuffer(pangolin::GlArrayBuffer, color_data_gray_);
  }
}

