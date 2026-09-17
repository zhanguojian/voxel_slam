#pragma once

#include "utils/common_lib.h"
#include <Eigen/Dense>
#include <omp.h>

#include <unordered_map>


#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;


struct VoxelMapConfig {
  double max_voxel_size_ = 0.5;
  int max_layer_ = 2;
  int max_iterations_ = 5;
  std::vector<int> layer_init_num_{5, 5, 5};
  int max_points_num_ = 50;
  double planner_threshold_ = 0.01;
  double beam_err_ = 0.02;
  double dept_err_ = 0.05;
  double sigma_num_ = 3.0;
  double sliding_thresh = 8.0;
  bool map_sliding_en = true;
  int half_map_size = 100;
};


typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{
public:
  VoxelOctoTree() = default;
  VoxelOctoTree(const VoxelOctoTree&) = delete;
  VoxelOctoTree& operator=(const VoxelOctoTree&) = delete;


  std::vector<pointWithVar> temp_points_;

  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  void init_octo_tree();
  void cut_octo_tree();
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
};

class VoxelMapManager
{
public:
  VoxelMapManager() = delete;
  VoxelMapManager(const VoxelMapManager&) = delete;
  VoxelMapManager& operator=(const VoxelMapManager&) = delete;
  ~VoxelMapManager();

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
  };

  void StateEstimation(StatesGroup &state_propagat);

  // void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
  //                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);
  void PreparePoints();

  void BuildVoxelMap();

  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  void build_single_residual(const pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void mapSliding();
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:

  VoxelMapConfig config_setting_;

  int current_frame_id_ = 0;

  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;

  M3D extR_ = M3D::Identity();
  V3D extT_ = V3D::Zero();

  // int scan_count = 0;

  StatesGroup state_;


  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  int feats_down_size_;
  int effct_feat_num_;
  
  // std::vector<M3D> cross_mat_list_;
  // std::vector<M3D> body_cov_list_;

  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

};

typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

