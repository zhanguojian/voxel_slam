
#pragma once

// ROS2 核心
#include <rclcpp/rclcpp.hpp>


// ROS2 消息类型
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>


// ROS2 TF
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>



/*** Livox ***/
namespace livox_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t tag;
  uint8_t line;
  double curvature;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace livox_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(double, curvature, curvature))
/****************/

/// @brief 点云消息类型
using PointCloudMsg = livox_ros::PointCloud2;
using PointCloudMsgPtr = PointCloudMsg::SharedPtr;

using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointCloud2MsgPtr = PointCloud2Msg::SharedPtr;


///// @brief IMU消息类型
using ImuMsg = sensor_msgs::msg::Imu;
using ImuMsgPtr = ImuMsg::SharedPtr;

/// @brief 字符串消息类型
using StrMsg = std_msgs::msg::String;
using StrMsgPtr = StrMsg::SharedPtr;


/// @brief 发布器指针
/// @tparam MessageT 发布器消息类型
template <typename MessageT>
using PublisherPtr = std::shared_ptr<rclcpp::Publisher<MessageT>>;


typedef pcl::PointXYZINormal PointType;

typedef pcl::PointCloud<PointType> PointCloudXYZI;

typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

typedef Eigen::Vector2f V2F;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};
