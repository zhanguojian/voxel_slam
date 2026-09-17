#include <gflags/gflags.h>
#include <glog/logging.h>
#include <rclcpp/time.hpp>

#include "yaml_config.hpp"


DEFINE_int32(glog_level, 0, "日志等级");
DEFINE_string(config_file, "../voxel.yaml", "配置文件路径");

int main(int argc, char** argv) {

    // 1.0 初始化gflags
    google::SetVersionString(version);
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr=true;

    int result=0;

    try {
        std::filesystem::path config="config/voxel.yaml";

        // 默认限制线程数，可用 OMP_NUM_THREADS 覆盖。
        if(!std::getenv("OMP_NUM_THREADS")) omp_set_num_threads(std::min(4,omp_get_max_threads()));

        Mapper mapper(loadConfig(config));
        mapper.run();
        
    } catch(const std::exception& e) 
    {
        LOG(ERROR)<<e.what(); result=1;
    }

    google::ShutdownGoogleLogging();
    return result;
}