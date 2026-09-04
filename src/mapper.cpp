#include "mapper.h"
#include "io.h"
#include <atomic>
#include <mutex>
#include <thread>
#include <limits>

Mapper::Mapper() : extT(0, 0, 0),  extR(M3D::Identity())
{
    extrinT.assign(3, 0.0);
    extrinR.assign(9, 0.0);

    p_imu.reset(new ImuProcess());

    VoxelMapConfig voxel_config;
    loadVoxelConfig(this->node, voxel_config);

    feats_undistort.reset(new PointCloudXYZI());
    feats_down_body.reset(new PointCloudXYZI());
    feats_down_world.reset(new PointCloudXYZI());

    pcl_w_wait_pub.reset(new PointCloudXYZI());
    pcl_wait_pub.reset(new PointCloudXYZI());
    pcl_wait_save.reset(new PointCloudXYZRGB());
    pcl_wait_save_intensity.reset(new PointCloudXYZI());

    voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
}

Mapper::~Mapper() {}

void Mapper::setComponentParams()
{

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    extT << VEC_FROM_ARRAY(extrinT);
    extR << MAT_FROM_ARRAY(extrinR);

    // 1. 设置IMU参数
    p_imu->set_extrinsic(extT, extR);
    p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_inv_expo_cov(inv_expo_cov);
    p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
    p_imu->set_imu_init_frame_num(imu_int_frame);

    // 2. 设置VoxelMapManager参数

    voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
    voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);
    voxelmap_manager->setFilterSizeSurfMin(filter_size_surf_min);
    voxelmap_manager->setFilterSizePcd(filter_size_pcd);
    voxelmap_manager->setGridSize(grid_size);
    voxelmap_manager->setPatchSize(patch_size);
    voxelmap_manager->setGridNWidth(grid_n_width);           
    
    if (!imu_en) p_imu->disable_imu();
    if (!gravity_est_en) p_imu->disable_gravity_est();
    if (!ba_bg_est_en) p_imu->disable_bias_est();
}

void Mapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = Measures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    LOG(INFO) << "FIRST LIDAR FRAME!" ;
  }
}

void Mapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    LOG(INFO) << "Gravity Alignment Starts";
    V3D ez(0, 0, -1), gz(_state.gravity);
    Eigen::Quaterniond G_q_I0 = Eigen::Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    LOG(INFO) << "Gravity Alignment Finished" 
              << " | Gravity: " << _state.gravity.transpose() 
              << " | Position: " << _state.pos_end.transpose() 
              << " | Velocity: " << _state.vel_end.transpose();
  }
}


void Mapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(Measures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}



bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  // PCAP 模式下现在有两个独立 producer：
  //   pcap_thread  -> lid_raw_data_buffer / imu_buffer
  //   image_thread -> img_buffer / img_time_buffer
  // 主线程在这里同时读取并 pop，因此一次同步决策期间统一持有 mtx_buffer。
  std::unique_lock<std::mutex> lock(mtx_buffer);


  case ONLY_LIO:
  {
    // ONLY_LIO 当前步骤必须有 LiDAR；启用 IMU 时还必须有 IMU。
    if (lid_raw_data_buffer.empty() || lid_header_time_buffer.empty())
    {
      return false;
    }

    if (imu_en && imu_buffer.empty())
    {
      return false;
    }

    if (meas.last_lio_update_time < 0.0)
    {
      meas.last_lio_update_time = lid_header_time_buffer.front();
    }

    if (!lidar_pushed)
    {
      meas.lidar = lid_raw_data_buffer.front();

      if (!meas.lidar || meas.lidar->points.size() <= 1)
      {
        // 无效帧直接丢掉，否则会永远卡在同一帧。
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
        sig_buffer.notify_all();
        return false;
      }

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();
      meas.lidar_frame_end_time =
        meas.lidar_frame_beg_time +
        meas.lidar->points.back().curvature / 1000.0;

      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;
    }

    // 等待 IMU 时间覆盖当前 LiDAR 帧末尾。
    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    {
      return false;
    }

    struct MeasureGroup m;
    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;

    while (imu_en && !imu_buffer.empty())
    {
      if (stamp2Sec(imu_buffer.front()->header.stamp) >
          meas.lidar_frame_end_time)
      {
        break;
      }

      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }

    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();

    meas.lio_vio_flg = LIO;
    meas.measures.push_back(m);
    lidar_pushed = false;

    sig_buffer.notify_all();
    return true;
  }

    case LIO:
    {
      // LIO -> VIO 只消费刚才用于 LIO 时间切分的同一张图。
      // 这一阶段不再强制要求 LiDAR/IMU buffer 非空，因此离线数据结尾
      // 即使 PCAP 已经读完，也能完成最后一次 VIO。
      if (img_buffer.empty() || img_time_buffer.empty())
      {
        return false;
      }

      const double img_capture_time =
        img_time_buffer.front() + exposure_time_init;

      meas.lio_vio_flg = VIO;
      meas.measures.clear();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();

      img_buffer.pop_front();
      img_time_buffer.pop_front();

      meas.measures.push_back(m);
      lidar_pushed = false;

      sig_buffer.notify_all();
      return true;
    }

    default:
      return false;
    }
  }

}

void Mapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    LOG(ERROR) << "[ LIO ]: No point!!!";
    return;
  }

    double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
    double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

    if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }


  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));


  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }


  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}



void Mapper::run()
{
  
    std::atomic<bool> pcap_finished{false};

    // -----------------------------------------------------------------------
    // Producer 1: PCAP -> LiDAR / IMU
    // -----------------------------------------------------------------------
    std::thread pcap_thread([this, &pcap_finished]() {
      RCLCPP_INFO(this->node->get_logger(), "开始读取 PCAP LiDAR/IMU");

      PcapIO pcap(pcap_file);


        pcap.addPointCloud2Handle(
          lid_topic,
          [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr &m) {
            standard_pcl_cbk(m);
            return true;
          });


        pcap.addIMUHandle(
          imu_topic,
          [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {
            imu_cbk(m);
            return true;
          });
      

      pcap.go();

      pcap_finished.store(true, std::memory_order_release);
      RCLCPP_INFO(this->node->get_logger(), "PCAP LiDAR/IMU 读取完成");
      sig_buffer.notify_all();
    });

    rclcpp::Rate rate(5000);

    while (rclcpp::ok())
    {
      if (!sync_packages(LidarMeasures))
      {
        const bool pcap_done =
          pcap_finished.load(std::memory_order_acquire);


        if (pcap_done && image_done)
        {
          LOG(INFO)<<"PCAP/RAW producer 已全部结束，当前 buffer 无法继续组成测量组，退出离线处理";
          break;
        }

        rate.sleep();
        continue;
      }

      handleFirstFrame();
      processImu();
      handlelio();
    }

    if (pcap_thread.joinable())
    {
      pcap_thread.join();
    }


    savePCD();
    return;
}

