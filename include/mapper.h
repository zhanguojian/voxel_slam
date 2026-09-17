#pragma once

#include "imu_process.h"
#include <fstream>
#include <functional>
#include <unordered_map>


class Mapper
{
    public:
        explicit Mapper(const RunConfig& config);
        ~Mapper();

        void run();



    private:

        bool sync_packages(MeasureGroup &meas)

        void savePCD();

        void process(const MeasureGroup& measures);

        struct MapCell { V3D sum=V3D::Zero(); double intensity=0; uint64_t count=0; };

        RunConfig config_;

        MeasureGroup LidarMeasures;
        StatesGroup _state;
        StatesGroup  state_propagat;

        ImuProcess imu_processor_;
        
        VoxelMapManagerPtr voxelmap_manager;

        StatesGroup state_;


        std::deque<LidarFrame> lidar_buffer_;
        std::deque<ImuSample> imu_buffer_;

        std::unordered_map<VOXEL_LOCATION,MapCell> output_map_;

        std::filesystem::path run_directory_;

        std::ofstream trajectory_;

        bool map_initialized_=false;

        size_t processed_=0, skipped_=0, unmatched_=0;

        std::function<void(PointCloudXYZI::ConstPtr,const StatesGroup&)> display_;


        PointCloudXYZI::Ptr feats_undistort;
        PointCloudXYZI::Ptr feats_down_body;


};

