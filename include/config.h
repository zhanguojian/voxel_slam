#pragma once
#include "voxel_map.h"
#include <filesystem>

struct RunConfig {

  std::filesystem::path source_path;
  std::filesystem::path output_path;

  int pointcloud_port = 56301;
  int imu_port = 56401;

  double frame_duration = 0.1;
  double blind = 0.5;
  double max_range = 100.0;
  int point_filter_num = 1;

  int tag_mask = 0;
  int tag_value = 0;
  double filter_size = 0.2;
  double save_voxel_size = 0.1;

  double gyro_noise = 0.01;
  double accel_noise = 0.1;
  double gyro_bias_noise = 0.0001;
  double accel_bias_noise = 0.0001;

  double max_imu_gap = 0.05;
  int imu_init_samples = 200;

  double init_gyro_std = 0.05;
  double init_accel_std = 0.5;

  bool gravity_align = true;

  bool viewer = false;
  bool save_scans = false;

  int viewer_scans = 100;

  int max_pending_frames = 100;
  int max_imu_samples = 200000;

  V3D extT = V3D::Zero();
  M3D extR = M3D::Identity();
  
  VoxelMapConfig voxel;
};
RunConfig loadConfig(const std::filesystem::path& file);
