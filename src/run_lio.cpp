#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/time.hpp>

#include "yaml_config.hpp"

int main(int argc, char** argv) {


    // 1.0 初始化gflags
    std::string version(SLAM_TOOLS_VERSION);
    google::SetVersionString(version);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // 2.0 配置读取
    YamlConfig yaml_config(FLAGS_config_file);
    RunConfig config;
    {
    config.source_path_ = SLAM_TOOLS_DATA_PATH + yaml_config.get("source_path", std::string("/bag/"));
    config.imu_topic_ = yaml_config.get("imu_topic", std::string("/livox/imu"));
    config.lidar_topic_ = yaml_config.get("lidar_topic", std::string("/livox/lidar"));
    config.pointcloud_port_ = yaml_config.get("pointcloud_port", 56301);
    config.imu_port_ = yaml_config.get("imu_port", 56401);

    LOG(INFO) << "source path: " << config.source_path_;
    LOG(INFO) << "imu topic: " << config.imu_topic_;
    LOG(INFO) << "cloud topic: " << config.lidar_topic_;
    }