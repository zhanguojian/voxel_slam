#pragma once

#include "imu_process.h"

class Mapper
{
    public:
        Mapper();
        ~Mapper();

        void initializeFiles();

        void Process(const MeasureGroup &meas, StatesGroup &state_inout);

        void gravityAlignment();
        void handleFirstFrame();

        void setComponentParams();

        bool sync_packages(MeasureGroup &meas);

        void handleLIO() 

        void run();
        void savePCD();
        void processImu();

        void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
        void imu_prop_callback();

        void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
        void pointBodyToWorld(const PointType &pi, PointType &po);
        
        // void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
        // void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr &msg_in);
        // void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr &msg_in);

        template <typename T> void set_posestamp(T &out);
        template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
        template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);



    private:

        pcl::VoxelGrid<PointType> downSizeFilterSurf;

        V3D euler_cur;

        MeasureGroup LidarMeasures;
        StatesGroup _state;
        StatesGroup  state_propagat;

        ImuProcess imu_processor_;
        
        VoxelMapManagerPtr voxelmap_manager;

        std::mutex mtx_buffer, mtx_buffer_imu_prop;

        std::condition_variable sig_buffer;

        
        string root_dir;
        string lid_topic, imu_topic;

        V3D extT;
        M3D extR;

        StatesGroup imu_propagate, latest_ekf_state;


        double res_mean_last = 0.05;

        double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;

        double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;


        bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;


    
    private:


        std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;

        int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;

        double filter_size_surf_min = 0;
        double filter_size_pcd = 0;
        double _first_lidar_time = 0.0;

        int feats_down_size = 0, max_iterations = 0;


        PointCloudXYZI::Ptr visual_sub_map;
        PointCloudXYZI::Ptr feats_undistort;
        PointCloudXYZI::Ptr feats_down_body;
        PointCloudXYZI::Ptr feats_down_world;
        PointCloudXYZI::Ptr pcl_w_wait_pub;
        PointCloudXYZI::Ptr pcl_wait_pub;
        PointCloudXYZRGB::Ptr pcl_wait_save;
        PointCloudXYZI::Ptr pcl_wait_save_intensity;

    private:

        int frame_num = 0;
        double aver_time_consu = 0;
        double aver_time_icp = 0;
        double aver_time_map_inre = 0;

        std::string pcap_file;
};

