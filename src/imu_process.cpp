#include "imu_process.h"

#include <rcpputils/asserts.hpp>

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }



bool ImuProcess::initialize(const MeasureGroup& m, StatesGroup& s) 
{

  for(const auto& sample:m.imu) 
  {
    if(sample.time<=last_init_time_ || sample.time>m.lidar.end) continue;

    last_init_time_=sample.time;

    ++init_count_;

    const V3D da=sample.accel-mean_acc_, dg=sample.gyro-mean_gyr_;

    mean_acc_+=da/init_count_; mean_gyr_+=dg/init_count_;

    m2_acc_+=da.cwiseProduct(sample.accel-mean_acc_);

    m2_gyr_+=dg.cwiseProduct(sample.gyro-mean_gyr_);
  }

  if(init_count_<config_.imu_init_samples)
  {
    return false;
  }

  if(std::sqrt(m2_acc_.sum()/(init_count_-1))>config_.init_accel_std ||
     std::sqrt(m2_gyr_.sum()/(init_count_-1))>config_.init_gyro_std ||
     mean_gyr_.norm()>0.2 || std::abs(mean_acc_.norm()-G_m_s2)>2.0)
  {

    LOG(ERROR)<<"IMU initialization is not stationary or has wrong acceleration units; restart with a stationary segment";
  }


  s.bias_g  = mean_gyr_;

  s.bias_a.setZero(); // 静止均值不能独立区分所有加计偏置与重力方向。

  if(config_.gravity_align) 
  {
    s.rot_end=Eigen::Quaterniond::FromTwoVectors(mean_acc_,V3D(0,0,G_m_s2)).toRotationMatrix();
    s.gravity=V3D(0,0,-G_m_s2);
  } else 
  {
    s.rot_end.setIdentity();
    s.gravity=-G_m_s2*mean_acc_.normalized();
  }

  s.cov.setIdentity(); 
  s.cov*=0.01;

  s.cov.block<3,3>(9,9)=M3D::Identity()*1e-5;
  s.cov.block<3,3>(12,12)=M3D::Identity()*1e-4;
  s.cov.block<3,3>(15,15)=M3D::Identity()*1e-4;

  state_time_=m.lidar.end;

  initialized_=true;

  LOG(INFO)<<"IMU initialized: samples="<<init_count_<<", gravity="<<s.gravity.transpose()
           <<", gyro bias="<<s.bias_g.transpose();
  return true;
}




//待修正
void ImuProcess::Forward_without_imu(MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
    LOG(WARNING) << "Forward_without_imu  " ;

    pcl_out = *(meas.lidar);

    /*** sort point clouds by offset time ***/
    const double &pcl_beg_time = meas.lidar_frame_beg_time;
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
    meas.last_lio_update_time = pcl_end_time;

    const double &pcl_end_offset_time = pcl_out.points.back().curvature / double(1000);

    MD(DIM_STATE, DIM_STATE) F_x, cov_w;
    double dt = 0;

    if (b_first_frame)
    {
        dt = 0.1;
        b_first_frame = false;
    }
    else { dt = pcl_beg_time - time_last_scan; }

    time_last_scan = pcl_beg_time;

    M3D Exp_f = Exp(state_inout.bias_g, dt);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
    F_x.block<3, 3>(0, 10) = Eye3d * dt;
    F_x.block<3, 3>(3, 7) = Eye3d * dt;

    cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt; // for omega in constant model
    cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt; // for velocity in constant model

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    state_inout.rot_end = state_inout.rot_end * Exp_f;
    state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;


    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;


    for(; it_pcl != pcl_out.points.begin(); it_pcl--)
    {
    dt_j= pcl_end_offset_time - it_pcl->curvature/double(1000);

    M3D R_jk(Exp(state_inout.bias_g, - dt_j));
    V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);


    // Using rotation and translation to un-distort points
    V3D p_jk;
    p_jk = - state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

    V3D P_compensate =  R_jk * P_j + p_jk;

    /// save Undistorted points and their rotation
    it_pcl->x = P_compensate(0);
    it_pcl->y = P_compensate(1);
    it_pcl->z = P_compensate(2);
    }

}


void ImuProcess::UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
    double t0 = omp_get_wtime();
    pcl_out.clear();

    MeasureGroup &meas = lidar_meas.measures.back();

    auto v_imu = meas.imu;
    v_imu.push_front(last_imu);
    const double &imu_beg_time = stamp2Sec(v_imu.front()->header.stamp);
    const double &imu_end_time = stamp2Sec(v_imu.back()->header.stamp);
    const double prop_beg_time = last_prop_end_time;

    const double prop_end_time = meas.lio_time ;

    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

      /*** forward propagation at each imu point ***/
    V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    // cout << "[ IMU ] input state: " << state_inout.vel_end.transpose() << " " << state_inout.pos_end.transpose() << endl;
    M3D R_imu(state_inout.rot_end);
    MD(DIM_STATE, DIM_STATE) F_x, cov_w;
    double dt, dt_all = 0.0;
    double offs_t;
    // double imu_time;
    double tau;
    if (!imu_time_init)
    {
        // imu_time = stamp2Sec(v_imu.front()->header.stamp) - first_lidar_time;
        // tau = 1.0 / (0.25 * sin(2 * CV_PI * 0.5 * imu_time) + 0.75);
        tau = 1.0;
        imu_time_init = true;
    }
    else
    {
        tau = state_inout.inv_expo_time;
        // RCLCPP_ERROR_STREAM(rclcpp::get_logger(""),"tau: %.6f !!!!!!", tau);
    }


    state_inout.vel_end = vel_imu;
    state_inout.rot_end = R_imu;
    state_inout.pos_end = pos_imu;
    state_inout.inv_expo_time = tau;

    last_imu = v_imu.back();
    last_prop_end_time = prop_end_time;

    double t1 = omp_get_wtime();

    if (pcl_wait_proc.points.size() < 1) return;


    auto it_pcl = pcl_wait_proc.points.end() - 1;
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
      auto head = it_kp - 1;
      auto tail = it_kp;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      // printf("head->offset_time: %lf \n", head->offset_time);
      // printf("it_pcl->curvature: %lf pt dt: %lf \n", it_pcl->curvature,
      // it_pcl->curvature / double(1000) - head->offset_time);

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
      {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        /* Transform to the 'end' frame */
        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // V3D P_compensate = Lid_rot_to_IMU.transpose() *
        // (state_inout.rot_end.transpose() * (R_i * (Lid_rot_to_IMU * P_i +
        // Lid_offset_to_IMU) + T_ei) - Lid_offset_to_IMU);
        V3D P_compensate = (extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - exrR_extT);

        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();

}


void ImuProcess::Process(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();
  rcpputils::assert_true(lidar_meas.lidar != nullptr);
  if (!imu_en)
  {
    Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
    return;
  }

  MeasureGroup meas = lidar_meas.measures.back();


  if (imu_need_init)
  {
    double pcl_end_time = lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;
    // lidar_meas.last_lio_update_time = pcl_end_time;

    if (meas.imu.empty()) { return; };
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init = true;

    last_imu = meas.imu.back();
    if (init_iter_num > MAX_INI_COUNT)
    {
      // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init = false;
      LOG(INFO) << "IMU Initials: Gravity: " << stat.gravity[0] << " " << stat.gravity[1] << " " << stat.gravity[2] << "; acc covarience: "
               << cov_acc[0] << " " << cov_acc[1] << " " << cov_acc[2] << "; gry covarience: " << cov_gyr[0] << " " << cov_gyr[1] << " " << cov_gyr[2];
      LOG(INFO) << "IMU Initials: ba covarience: " << cov_bias_acc[0] << " " << cov_bias_acc[1] << " " << cov_bias_acc[2] << "; bg covarience: "
               << cov_bias_gyr[0] << " " << cov_bias_gyr[1] << " " << cov_bias_gyr[2];

    }

    return;
  }


  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
  // cout << "[ IMU ] undistorted point num: " << cur_pcl_un_->size() << endl;
}