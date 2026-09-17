#pragma once

#include "config.h"

extern const bool time_list(PointType &x, PointType &y);


/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ImuProcess(const RunConfig& config) : config_(config) {}
  ~ImuProcess();

  // 初始化阶段返回 false；异常时间覆盖或运动初始化会报错，不伪造 IMU 数据。

  bool Process(MeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);

  void UndistortPcl(MeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  void Forward_without_imu(MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);


  bool initialized() const { return initialized_; }
  double stateTime() const { return state_time_; }

private:

  struct PoseSample {
    double time=0;
    M3D rotation=M3D::Identity();
    V3D position=V3D::Zero(); 
    velocity=V3D::Zero(); 
    acceleration=V3D::Zero(); 
    omega=V3D::Zero();
  };

  bool initialize(const MeasureGroup&, StatesGroup&);

  RunConfig config_;

  bool initialized_=false;

  int init_count_=0;

  double last_init_time_=-1;

  double state_time_=-1;

  V3D mean_acc_=V3D::Zero();
  mean_gyr_=V3D::Zero();
  V3D m2_acc_=V3D::Zero(); 
  m2_gyr_=V3D::Zero();
};

typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
