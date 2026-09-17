

#include "voxel_map.h"

using namespace Eigen;


//根据激光雷达一个点 pb 的“距离误差 + 角度误差”，估计这个点在 LiDAR/Body 坐标系下的 3×3 位置协方差 cov。
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov)
{
  if (pb[2] == 0) pb[2] = 0.0001;
  
  //距离
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);

  //距离测量标准差。
  float range_var = range_inc * range_inc;

    // 定义角度协方差
  Eigen::Matrix2d direction_var;
  //一个三维方向只需要两个独立角度描述，比如：，水平角 azimuth,垂直角 elevation，水平角误差和垂直角误差大小相同，并且二者互不相关。
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

    
  Eigen::Vector3d direction(pb);
  //归一化
  direction.normalize();

    //构造方向向量的反对称矩阵
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;

  //快速构造第一个切向量，点积相乘为零，保证垂直于激光方向向量 direction。
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    //归一化第一个基向量
  base_vector1.normalize();
    // 构造第二个切向量
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    //归一化第二个基向量
  base_vector2.normalize();

    //构造 N 矩阵,激光方向 u 垂直平面上的二维正交基。
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);

  //构造 A 矩阵,角度误差 → XYZ 点位置误差的雅可比矩阵。越远的 LiDAR 点，角度误差导致的位置不确定性越大。
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

void loadVoxelConfig(VoxelMapConfig &voxel_config)
{
  // declare parameter
   <bool>("publish.pub_plane_en", false);
   <int>("lio.max_layer", 1);
   <double>("lio.voxel_size", 0.5);
   <double>("lio.min_eigen_value", 0.01);
   <double>("lio.sigma_num", 3);
   <double>("lio.beam_err", 0.02);
   <double>("lio.dept_err", 0.05);

   <vector<int64_t>>("lio.layer_init_num", std::vector<int64_t>{5,5,5,5,5}); 
   <int>("lio.max_points_num", 50);
   <int>("lio.min_iterations", 5);
   <bool>("local_map.map_sliding_en", false);
   <int>("local_map.half_map_size", 100);
   <double>("local_map.sliding_thresh", 8.0);
}

//输入带协方差的点云，判断输出点云是否为平面点云，并计算平面法向量、平面中心、平面协方差矩阵、平面误差协方差矩阵等。
void VoxelOctoTree::init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane)
{
    // 平面的不确定性由平面法向量，平面中心组成六维向量，平面误差协方差矩阵是 6×6 的。
    //清空平面信息，plane_var_ 估计出来的 plane 参数有多不确定，用于概率残差
  plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();

  //covariance是当前 voxel 里面所有点的位置分布协方差。，用来 PCA 判断是不是平面
  plane->covariance_ = Eigen::Matrix3d::Zero();


  plane->center_ = Eigen::Vector3d::Zero();
  plane->normal_ = Eigen::Vector3d::Zero();
  plane->points_size_ = points.size();
  
  //平面半径初始化。
  plane->radius_ = 0;

  //求质心和协方差
  for (const auto &pv : points) 
  {
    plane->covariance_ += pv.point_w * pv.point_w.transpose();
    plane->center_ += pv.point_w;
   }
  plane->center_ = plane->center_ / plane->points_size_;
  plane->covariance_ = plane->covariance_ / plane->points_size_ - plane->center_ * plane->center_.transpose();

   //对协方差矩阵做特征值分解，最小特征值对应的特征向量就是平面法向量。
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
  //保存三个特征向量。
  Eigen::Matrix3cd evecs = es.eigenvectors();
  //保存三个特征值。
  Eigen::Vector3cd evals = es.eigenvalues();
  //取特征值实部
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();


  Eigen::Matrix3f::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;

  //最小特征值对应的方向。
  Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
  Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
  Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

  //算 center 对点坐标的 Jacobian
  Eigen::Matrix3d J_Q;
  J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_;


  // && evalsReal(evalsMid) > 0.05
  //&& evalsReal(evalsMid) > 0.01
  //判断是不是平面
  if (evalsReal(evalsMin) < planer_threshold_)
  {
    //这个点的位置误差会让 plane normal 和 center 改变多少。
    for (int i = 0; i < points.size(); i++)
    {
      Eigen::Matrix<double, 6, 3> J;//雅可比矩阵，前 3 行在算 normal 对点坐标的 Jacobian，后 3 行在算 center 对点坐标的 Jacobian。
        //特征向量对点位置扰动的导数中间量。算“点的位置变化，会让平面法向量变化多少”。
      Eigen::Matrix3d F;
        //遍历三个特征方向，因为最小特征向量 normal 的变化，会受到另外两个特征方向的影响。
      for (int m = 0; m < 3; m++)
      {
        //单位特征向量自身方向的变化并不通过自己这项表示
        if (m != (int)evalsMin)
        {
            //当前 point_i 在 xyz 上发生小变化时，会让 normal 沿第 m 个特征方向变化多少。
          Eigen::Matrix<double, 1, 3> F_m =
              (points[i].point_w - plane->center_).transpose() / ((plane->points_size_) * (evalsReal[evalsMin] - evalsReal[m])) *
              (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() + evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
          F.row(m) = F_m;
        }
        else
        {
          Eigen::Matrix<double, 1, 3> F_m;
          F_m << 0, 0, 0;
          F.row(m) = F_m;
        }
      }
      //前 3 行在算 normal 对点坐标的 Jacobian，后 3 行在算 center 对点坐标的 Jacobian。
      J.block<3, 3>(0, 0) = evecs.real() * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_var_ += J * points[i].var * J.transpose();
    }

    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) + plane->normal_(1) * plane->center_(1) + plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    plane->is_update_ = true;
    if (!plane->is_init_)
    {
      plane->id_ = voxel_plane_id;
      voxel_plane_id++;
      plane->is_init_ = true;
    }
  }
  else
  {
    plane->is_update_ = true;
    plane->is_plane_ = false;
  }
}

void VoxelOctoTree::init_octo_tree()
{
  if (temp_points_.size() > points_size_threshold_)
  {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_ == true)
    {
      octo_state_ = 0;
      // new added
      if (temp_points_.size() > max_points_num_)
      {
        update_enable_ = false;
        std::vector<pointWithVar>().swap(temp_points_);
        new_points_ = 0;
      }
    }
    else
    {
      octo_state_ = 1;
      cut_octo_tree();
    }
    init_octo_ = true;
    new_points_ = 0;
  }
}

void VoxelOctoTree::cut_octo_tree()
{
  if (layer_ >= max_layer_)
  {
    octo_state_ = 0;
    return;
  }
  for (size_t i = 0; i < temp_points_.size(); i++)
  {
    int xyz[3] = {0, 0, 0};
    if (temp_points_[i].point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
    if (temp_points_[i].point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
    if (temp_points_[i].point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] == nullptr)
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
    }
    leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
    leaves_[leafnum]->new_points_++;
  }
  for (uint i = 0; i < 8; i++)
  {
    if (leaves_[i] != nullptr)
    {
      if (leaves_[i]->temp_points_.size() > leaves_[i]->points_size_threshold_)
      {
        init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
        if (leaves_[i]->plane_ptr_->is_plane_)
        {
          leaves_[i]->octo_state_ = 0;
          // new added
          if (leaves_[i]->temp_points_.size() > leaves_[i]->max_points_num_)
          {
            leaves_[i]->update_enable_ = false;
            std::vector<pointWithVar>().swap(leaves_[i]->temp_points_);
            new_points_ = 0;
          }
        }
        else
        {
          leaves_[i]->octo_state_ = 1;
          leaves_[i]->cut_octo_tree();
        }
        leaves_[i]->init_octo_ = true;
        leaves_[i]->new_points_ = 0;
      }
    }
  }
}

void VoxelOctoTree::UpdateOctoTree(const pointWithVar &pv)
{
  if (!init_octo_)
  {
    new_points_++;
    temp_points_.push_back(pv);
    if (temp_points_.size() > points_size_threshold_) { init_octo_tree(); }
  }
  else
  {
    if (plane_ptr_->is_plane_)
    {
      if (update_enable_)
      {
        new_points_++;
        temp_points_.push_back(pv);
        if (new_points_ > update_size_threshold_)
        {
          init_plane(temp_points_, plane_ptr_);
          new_points_ = 0;
        }
        if (temp_points_.size() >= max_points_num_)
        {
          update_enable_ = false;
          std::vector<pointWithVar>().swap(temp_points_);
          new_points_ = 0;
        }
      }
    }
    else
    {
      if (layer_ < max_layer_)
      {
        int xyz[3] = {0, 0, 0};
        if (pv.point_w[0] > voxel_center_[0]) { xyz[0] = 1; }
        if (pv.point_w[1] > voxel_center_[1]) { xyz[1] = 1; }
        if (pv.point_w[2] > voxel_center_[2]) { xyz[2] = 1; }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) { leaves_[leafnum]->UpdateOctoTree(pv); }
        else
        {
          leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
          leaves_[leafnum]->layer_init_num_ = layer_init_num_;
          leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
          leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
          leaves_[leafnum]->quater_length_ = quater_length_ / 2;
          leaves_[leafnum]->UpdateOctoTree(pv);
        }
      }
      else
      {
        if (update_enable_)
        {
          new_points_++;
          temp_points_.push_back(pv);
          if (new_points_ > update_size_threshold_)
          {
            init_plane(temp_points_, plane_ptr_);
            new_points_ = 0;
          }
          if (temp_points_.size() > max_points_num_)
          {
            update_enable_ = false;
            std::vector<pointWithVar>().swap(temp_points_);
            new_points_ = 0;
          }
        }
      }
    }
  }
}

VoxelOctoTree *VoxelOctoTree::find_correspond(Eigen::Vector3d pw)
{
  if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_)) return this;

  int xyz[3] = {0, 0, 0};
  xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
  xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
  xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
  int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

  // printf("leafnum: %d. \n", leafnum);

  return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw) : this;
}

VoxelOctoTree *VoxelOctoTree::Insert(const pointWithVar &pv)
{
  if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) || (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_)))
  {
    new_points_++;
    temp_points_.push_back(pv);
    return this;
  }

  if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_))
  {
    int xyz[3] = {0, 0, 0};
    xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (leaves_[leafnum] != nullptr) { return leaves_[leafnum]->Insert(pv); }
    else
    {
      leaves_[leafnum] = new VoxelOctoTree(max_layer_, layer_ + 1, layer_init_num_[layer_ + 1], max_points_num_, planer_threshold_);
      leaves_[leafnum]->layer_init_num_ = layer_init_num_;
      leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
      leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
      leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      return leaves_[leafnum]->Insert(pv);
    }
  }
  return nullptr;
}

void VoxelMapManager::BuildVoxelMap()
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);

  std::vector<pointWithVar> input_points;

  for (size_t i = 0; i < feats_down_world_->size(); i++)
  {
    pointWithVar pv;
    pv.point_w << feats_down_world_->points[i].x, feats_down_world_->points[i].y, feats_down_world_->points[i].z;
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    //把当前 LiDAR 点的自身测量协方差，从 LiDAR/body 坐标系传播到世界坐标系，
    //同时把当前状态里的姿态不确定性和位置不确定性也加进去，得到这个世界坐标点最终的 3×3 协方差 var
    var = (state_.rot_end * extR_) * var * (state_.rot_end * extR_).transpose() +
          (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + state_.cov.block<3, 3>(3, 3);
    pv.var = var;
    input_points.push_back(pv);
  }

  uint plsize = input_points.size();

  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar &p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end())
    {
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
    }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->temp_points_.push_back(p_v);
      voxel_map_[position]->new_points_++;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
    }
  }
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter)
  {
    iter->second->init_octo_tree();
  }
}

void VoxelMapManager::UpdateVoxelMap(const std::vector<pointWithVar> &input_points)
{
  float voxel_size = config_setting_.max_voxel_size_;
  float planer_threshold = config_setting_.planner_threshold_;
  int max_layer = config_setting_.max_layer_;
  int max_points_num = config_setting_.max_points_num_;
  std::vector<int> layer_init_num = convertToIntVectorSafe(config_setting_.layer_init_num_);
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++)
  {
    const pointWithVar &p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = p_v.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);
    if (iter != voxel_map_.end()) { voxel_map_[position]->UpdateOctoTree(p_v); }
    else
    {
      VoxelOctoTree *octo_tree = new VoxelOctoTree(max_layer, 0, layer_init_num[0], max_points_num, planer_threshold);
      voxel_map_[position] = octo_tree;
      voxel_map_[position]->layer_init_num_ = layer_init_num;
      voxel_map_[position]->quater_length_ = voxel_size / 4;
      voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      voxel_map_[position]->UpdateOctoTree(p_v);
    }
  }
}



/*
                 当前帧点云
                     ↓
                pv_list_
                     ↓
           BuildResidualListOMP
                     ↓
                 ptpl_list_
                     ↓
      ┌─────────────────────────────┐
      │ 每个 point-to-plane        │
      │                             │
      │ residual                   │
      │ r = nᵀ(p-c)                │
      │                             │
      │ point variance             │
      │ σp² = nᵀΣp n               │
      │                             │
      │ plane variance             │
      │ σplane² = JΣplaneJᵀ        │
      │                             │
      │ R = 0.001+σp²+σplane²      │
      │                             │
      │ H = [Aᵀ  nᵀ]               │
      └─────────────────────────────┘
                     ↓
                  所有点
                     ↓
               HᵀR⁻¹H
               HᵀR⁻¹z
                     ↓
          + IMU prior P⁻¹
                     ↓
     (HᵀR⁻¹H + P⁻¹)⁻¹
                     ↓
                 solution
                     ↓
             state_ += solution
                     ↓
          rotation increment
          translation increment
                     ↓
              是否收敛？
               /      \
             否        是
             ↓          ↓
          下一轮      再确认匹配
                        ↓
                    最终停止
                        ↓
                更新 state_.cov
*/
void VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
{
    //清空上一帧保存的点叉乘矩阵。
  cross_mat_list_.clear();
  cross_mat_list_.reserve(feats_down_size_);
  //当前帧每个 LiDAR 点自己的 3×3 测量协方差。
  body_cov_list_.clear();
  body_cov_list_.reserve(feats_down_size_);

  // build_residual_time = 0.0;
  // ekf_time = 0.0;
  // double t0 = omp_get_wtime();


   // 预计算每个点的 covariance 和叉乘矩阵
  for (size_t i = 0; i < feats_down_body_->size(); i++)
  {
    V3D point_this(feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z);
    if (point_this[2] == 0) { point_this[2] = 0.001; }

    //计算这个点自己的测量 covariance,LiDAR 自己测这个点有多不准。
    M3D var;
    calcBodyCov(point_this, config_setting_.dept_err_, config_setting_.beam_err_, var);
    body_cov_list_.push_back(var);

    //位姿旋转扰动 Jacobian。
    point_this = extR_ * point_this + extT_;
    //构造叉乘矩阵
    M3D point_crossmat;
    //不要 IEKF 每轮迭代都重新构造。
    point_crossmat << SKEW_SYM_MATRX(point_this);
    cross_mat_list_.push_back(point_crossmat);
  }

  //清空上一帧的 vector，同时释放之前 capacity。
  vector<pointWithVar>().swap(pv_list_);
  pv_list_.resize(feats_down_size_);

  //初始化重新匹配次数,IEKF 已经收敛/重新匹配过多少次。
  int rematch_num = 0;

  //定义几个 IEKF 矩阵
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();



  //定义几个 IEKF 标志位
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;

  /* 开始 IEKF 迭代

    重新计算当前点世界位置
    ↓
    重新查Voxel Plane
    ↓
    重新构造残差
    ↓
    重新求位姿增量
  */
  for (int iterCount = 0; iterCount < config_setting_.max_iterations_; iterCount++)
  {
    double total_residual = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZI>);

    //每轮 IEKF 更新以后，点的位置都会根据新的 state_ 重新计算。
    TransformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    M3D rot_var = state_.cov.block<3, 3>(0, 0);
    M3D t_var = state_.cov.block<3, 3>(3, 3);

    for (size_t i = 0; i < feats_down_body_->size(); i++)
    {
      pointWithVar &pv = pv_list_[i];
      pv.point_b << feats_down_body_->points[i].x, feats_down_body_->points[i].y, feats_down_body_->points[i].z;
      pv.point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;

      M3D cov = body_cov_list_[i];
      M3D point_crossmat = cross_mat_list_[i];

      cov = state_.rot_end * cov * state_.rot_end.transpose() + (-point_crossmat) * rot_var * (-point_crossmat.transpose()) + t_var;

      pv.var = cov;
      pv.body_var = body_cov_list_[i];
    }

    //清空 point-to-plane 匹配结果
    ptpl_list_.clear();

    // double t1 = omp_get_wtime();

    BuildResidualListOMP(pv_list_, ptpl_list_);

    // build_residual_time += omp_get_wtime() - t1;

    //计算这一轮平均残差
    for (int i = 0; i < ptpl_list_.size(); i++)
    {
      total_residual += fabs(ptpl_list_[i].dis_to_plane_);
    }
    effct_feat_num_ = ptpl_list_.size();
    LOG(INFO) << "[ LIO ] Raw feature num: " << feats_undistort_->size() << ", downsampled feature num:" << feats_down_size_ 
         << " effective feature num: " << effct_feat_num_ << " average residual: " << total_residual / effct_feat_num_ << endl;

    /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
     * ***/
     //建立 H 矩阵和逆矩阵
    MatrixXd Hsub(effct_feat_num_, 6);
    MatrixXd Hsub_T_R_inv(6, effct_feat_num_);

    //建立测量逆方差
    VectorXd R_inv(effct_feat_num_);

    //建立 measurement vector
    VectorXd meas_vec(effct_feat_num_);
    meas_vec.setZero();

    //逐个处理 point-to-plane 匹配,构造完整测量模型
    for (int i = 0; i < effct_feat_num_; i++)
    {
      auto &ptpl = ptpl_list_[i];

      //取出雷达点转换为imu坐标系下点
      V3D point_this(ptpl.point_b_);
      point_this = extR_ * point_this + extT_;

      //该点的反对称矩阵
      V3D point_body(ptpl.point_b_);
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);

      /*** get the normal vector of closest surface/corner ***/

      //计算先验状态下的世界点
      V3D point_world = state_propagat.rot_end * point_this + state_propagat.pos_end;

      //构造平面参数误差 Jacobian
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list_[i].center_;
      J_nq.block<1, 3>(0, 3) = -ptpl_list_[i].normal_;

      M3D var;
      // point_body cov，LiDAR点自身测量误差旋转到世界坐标系。
      var = state_propagat.rot_end * extR_ * ptpl_list_[i].body_cov_ * (state_propagat.rot_end * extR_).transpose();

      //地图平面不确定性传播到 residual,平面自己不准确，造成点到平面 residual 有多大的方差。
      double sigma_l = J_nq * ptpl_list_[i].plane_var_ * J_nq.transpose();

      //计算每个测量权重:0.001 + 平面的方差 + 点沿法向量方向上的误差。
      R_inv(i) = 1.0 / (0.001 + sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);
      // R_inv(i) = 1.0 / (sigma_l + ptpl_list_[i].normal_.transpose() * var * ptpl_list_[i].normal_);

      /*** calculate the Measuremnt Jacobian matrix H，计算旋转 Jacobian，A就是 residual 对旋转小扰动的 3 维 Jacobian。 ***/
      V3D A(point_crossmat * state_.rot_end.transpose() * ptpl_list_[i].normal_);

      //构造 H 的第 i 行
      Hsub.row(i) << VEC_FROM_ARRAY(A), ptpl_list_[i].normal_[0], ptpl_list_[i].normal_[1], ptpl_list_[i].normal_[2];
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i), A[2] * R_inv(i), ptpl_list_[i].normal_[0] * R_inv(i),
          ptpl_list_[i].normal_[1] * R_inv(i), ptpl_list_[i].normal_[2] * R_inv(i);

        //构造 measurement residual
      meas_vec(i) = -ptpl_list_[i].dis_to_plane_;
    }

    //重置没有结束和收敛结束标志
    EKF_stop_flg = false;
    flg_EKF_converged = false;


    /*** Iterative Kalman Filter Update ，这个好像没有使用 ***/
    MatrixXd K(DIM_STATE, effct_feat_num_);


    // auto &&Hsub_T = Hsub.transpose();
    auto &&HTz = Hsub_T_R_inv * meas_vec;

    // fout_dbg<<"HTz: "<<HTz<<endl;LiDAR measurement 对 pose 提供的信息矩阵。当前这些平面约束到底把 XYZ 和旋转约束得有多强。
    H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;

    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));  K_1 可以理解成：当前这轮融合后的 posterior covariance 类矩阵。
    MD(DIM_STATE, DIM_STATE) &&K_1 = (H_T_H.block<DIM_STATE, DIM_STATE>(0, 0) + state_.cov.block<DIM_STATE, DIM_STATE>(0, 0).inverse()).inverse();

    G.block<DIM_STATE, 6>(0, 0) = K_1.block<DIM_STATE, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);

    //计算先验状态与当前迭代状态之间的差
    auto vec = state_propagat - state_;

    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);

    int minRow, minCol;

    //更新状态
    state_ += solution;
    auto vec = state_propagat - state_;

    VD(DIM_STATE)
    solution = K_1.block<DIM_STATE, 6>(0, 0) * HTz + vec.block<DIM_STATE, 1>(0, 0) - G.block<DIM_STATE, 6>(0, 0) * vec.block<6, 1>(0, 0);

    int minRow, minCol;

    //更新状态
    state_ += solution;

    //取旋转增量,平移增量
    auto rot_add = solution.block<3, 1>(0, 0);
    auto t_add = solution.block<3, 1>(3, 0);

    //判断收敛
    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) { flg_EKF_converged = true; }

    //获取 Euler angle
    V3D euler_cur = state_.rot_end.eulerAngles(2, 1, 0);

    /*** Rematch Judgement ***/
    //再次匹配
    if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (config_setting_.max_iterations_ - 2)))) { rematch_num++; }

    /*** Convergence Judgements and Covariance Update ***/
    //最终停止条件,正常收敛或者一直没收敛且到最后一轮
    if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == config_setting_.max_iterations_ - 1)))
    {
      /*** Covariance Update ***/
      // _state.cov = (I_STATE - G) * _state.cov;
      state_.cov.block<DIM_STATE, DIM_STATE>(0, 0) =
          (I_STATE.block<DIM_STATE, DIM_STATE>(0, 0) - G.block<DIM_STATE, DIM_STATE>(0, 0)) * state_.cov.block<DIM_STATE, DIM_STATE>(0, 0);
      // total_distance += (_state.pos_end - position_last).norm();

      //保存最终位置
      position_last_ = state_.pos_end;
      //生成 quaternion,返回的是 Z-Y-X 顺序相关角度，
      geoQuat_ = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));

      // VD(DIM_STATE) K_sum  = K.rowwise().sum();
      // VD(DIM_STATE) P_diag = _state.cov.diagonal();
      EKF_stop_flg = true;
    }
    if (EKF_stop_flg) break;
  }

  // double t2 = omp_get_wtime();
  // scan_count++;
  // ekf_time = t2 - t0 - build_residual_time;

  // ave_build_residual_time = ave_build_residual_time * (scan_count - 1) / scan_count + build_residual_time / scan_count;
  // ave_ekf_time = ave_ekf_time * (scan_count - 1) / scan_count + ekf_time / scan_count;

  // cout << "[ Mapping ] ekf_time: " << ekf_time << "s, build_residual_time: " << build_residual_time << "s" << endl;
  // cout << "[ Mapping ] ave_ekf_time: " << ave_ekf_time << "s, ave_build_residual_time: " << ave_build_residual_time << "s" << endl;
}

void VoxelMapManager::TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR_ * p + extT_) + t);
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void VoxelMapManager::BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list)
{
  int max_layer = config_setting_.max_layer_;
  double voxel_size = config_setting_.max_voxel_size_;
  double sigma_num = config_setting_.sigma_num_;
  //这是一个线程锁。
  std::mutex mylock;

  // clear the previous residual list
  ptpl_list.clear();

    //保存所有点的匹配结果，后面再根据标记筛选出匹配成功的点。
  std::vector<PointToPlane> all_ptpl_list(pv_list.size());
  //标记哪些点匹配成功
  std::vector<bool> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size());


    //for (size_t i = 0; i < pv_list.size(); ++i)
  for (size_t i = 0; i < index.size(); ++i)
  {
    index[i] = i;
    useful_ptpl[i] = false;
  }

  //OpenMP 宏控制
  #ifdef MP_EN
  //设置 OpenMP 使用多少线程。
    omp_set_num_threads(MP_PROC_NUM);
    //下面这个 for 循环可以把不同 i 分给不同 CPU 线程同时执行。实际上 OpenMP 如何划分由运行时决定。
    #pragma omp parallel for
  #endif


  //每个点独立找地图平面。
  for (int i = 0; i < index.size(); i++)
  {
    const pointWithVar &pv = pv_list[i];

    //该点的坐标
    float loc_xyz[3];
    for (int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pv.point_w[j] / voxel_size;
      if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
    }
    //构造体素坐标，当前点在顶层 Hash Voxel Map 中的 key。
    VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = voxel_map_.find(position);

    if (iter != voxel_map_.end())
    {
        //取当前八叉树
      VoxelOctoTree *current_octo = iter->second;

      //建立一个单点匹配结果
      PointToPlane single_ptpl;

      //初始化成功标记，默认没有匹配成功
      bool is_sucess = false;
      //匹配概率或评价分数。
      double prob = 0;

      build_single_residual(pv, current_octo, 0, is_sucess, prob, single_ptpl);
      if (!is_sucess)
      {
        VOXEL_LOCATION near_position = position;
        if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) { near_position.x = near_position.x + 1; }
        else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) { near_position.x = near_position.x - 1; }
        if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) { near_position.y = near_position.y + 1; }
        else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) { near_position.y = near_position.y - 1; }
        if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) { near_position.z = near_position.z + 1; }
        else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) { near_position.z = near_position.z - 1; }
        auto iter_near = voxel_map_.find(near_position);
        if (iter_near != voxel_map_.end() && !(near_position == position)) { build_single_residual(pv, iter_near->second, 0, is_sucess, prob, single_ptpl); }
      }
      if (is_sucess)
      {
        mylock.lock();
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        mylock.unlock();
      }
      else
      {
        mylock.lock();
        useful_ptpl[i] = false;
        mylock.unlock();
      }
    }
  }

  //把成功结果压缩到最终：ptpl_list
  for (size_t i = 0; i < useful_ptpl.size(); i++)
  {
    if (useful_ptpl[i]) { ptpl_list.push_back(all_ptpl_list[i]); }
  }
}

//进入这个 VoxelOctoTree 以后，究竟哪个平面能够和当前点形成有效匹配？如果有多个候选平面，选哪个？
void VoxelMapManager::build_single_residual(const pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess,
                                            double &prob, PointToPlane &single_ptpl)
{
  int max_layer = config_setting_.max_layer_;
  //N-sigma 概率门限判断。
  double sigma_num = config_setting_.sigma_num_;
    //平面半径放大系数,当前点在平面内的投影不能离这个局部平面中心太远
  double radius_k = 3;


  Eigen::Vector3d p_w = pv.point_w;

  //里面的点可以由一个局部平面表示。和当前点直接做匹配
  if (current_octo->plane_ptr_->is_plane_)
  {
    //取出平面
    const VoxelPlane &plane = *current_octo->plane_ptr_;
    // Eigen::Vector3d p_world_to_center = p_w - plane.center_;

    //就是点到平面的垂直距离。
    float dis_to_plane = fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_);

    //点到 plane center 的距离平方
    float dis_to_center = (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) + (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
                          (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));

    //把当前点投影到平面之后，投影点距离局部平面中心有多远
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    //平面范围门限,voxelane 不是无限大，在voxel size 限制中
    if (range_dis <= radius_k * plane.radius_)
    {
        //构造平面参数 Jacobian
      Eigen::Matrix<double, 1, 6> J_nq;

      //平面方程参数由法向量和中心点构建，j分写是n法向量和c中心点对r距离残差进行求导
      J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
      J_nq.block<1, 3>(0, 3) = -plane.normal_;

      //平面本身的不确定性,传播到点到平面这一维 residual 上。
      double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();

      // 再加当前点的不确定性,但点到平面 residual 只关心：法向方向上的误差。最终当前这个 point-to-plane residual 总共有多不确定
      sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;

        //N-sigma 残差门限
      if (dis_to_plane < sigma_num * sqrt(sigma_l))
      {
        //至少已经找到一个满足几何范围 + Nσ 检验的平面。
        is_sucess = true;

        //它同时考虑：残差有多小 + 预测的不确定性有多大
        double this_prob = 1.0 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);

        //比之前遇到的候选 plane 更靠谱。
        if (this_prob > prob)
        {
          prob = this_prob;
          //把当前点成功匹配到的地图平面法向量，记录到这个点的 normal 字段里。
          pv.normal = plane.normal_;
          single_ptpl.body_cov_ = pv.body_var;
          single_ptpl.point_b_ = pv.point_b;
          single_ptpl.point_w_ = pv.point_w;
          single_ptpl.plane_var_ = plane.plane_var_;
          single_ptpl.normal_ = plane.normal_;
          single_ptpl.center_ = plane.center_;
          single_ptpl.d_ = plane.d_;
          single_ptpl.layer_ = current_layer;
          single_ptpl.dis_to_plane_ = plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) + plane.normal_(2) * p_w(2) + plane.d_;
        }
        return;
      }
      else
      {
        // is_sucess = false;
        return;
      }
    }
    else
    {
      // is_sucess = false;
      return;
    }
  }
  else
  {
    //当前这一层不能用一个 plane 表达。还没有到最大层,就继续递归。
    if (current_layer < max_layer)
    {
      for (size_t leafnum = 0; leafnum < 8; leafnum++)
      {
        if (current_octo->leaves_[leafnum] != nullptr)
        {

          VoxelOctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, is_sucess, prob, single_ptpl);
        }
      }
      return;
    }
    else { return; }
  }
}

void VoxelMapManager::pubVoxelMap()
{
  double max_trace = 0.25;
  double pow_num = 0.2;
  rclcpp::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::msg::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<VoxelPlane> pub_plane_list;
  for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); iter++)
  {
    GetUpdatePlane(iter->second, config_setting_.max_layer_, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++)
  {
    V3D plane_cov = pub_plane_list[i].plane_var_.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) { trace = max_trace; }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane_) { alpha = use_alpha; }
    else { alpha = 0; }
    pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
  }
  voxel_map_pub_->publish(voxel_plane);
  loop.sleep();
}

void VoxelMapManager::GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list)
{
  if (current_octo->layer_ > pub_max_voxel_layer) { return; }
  if (current_octo->plane_ptr_->is_update_) { plane_list.push_back(*current_octo->plane_ptr_); }
  if (current_octo->layer_ < current_octo->max_layer_)
  {
    if (!current_octo->plane_ptr_->is_plane_)
    {
      for (size_t i = 0; i < 8; i++)
      {
        if (current_octo->leaves_[i] != nullptr) { GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list); }
      }
    }
  }
  return;
}

void VoxelMapManager::pubSinglePlane(visualization_msgs::msg::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane,
                                     const float alpha, const Eigen::Vector3d rgb)
{
  visualization_msgs::msg::Marker plane;
  plane.header.frame_id = "camera_init";
  plane.header.stamp = rclcpp::Time();
  plane.ns = plane_ns;
  plane.id = single_plane.id_;
  plane.type = visualization_msgs::msg::Marker::CYLINDER;
  plane.action = visualization_msgs::msg::Marker::ADD;
  plane.pose.position.x = single_plane.center_[0];
  plane.pose.position.y = single_plane.center_[1];
  plane.pose.position.z = single_plane.center_[2];
  geometry_msgs::msg::Quaternion q;
  CalcVectQuation(single_plane.x_normal_, single_plane.y_normal_, single_plane.normal_, q);
  plane.pose.orientation = q;
  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value_);
  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value_);
  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value_);
  plane.color.a = alpha;
  plane.color.r = rgb(0);
  plane.color.g = rgb(1);
  plane.color.b = rgb(2);
  plane.lifetime = rclcpp::Duration::from_seconds(0.01);
  plane_pub.markers.push_back(plane);
}

void VoxelMapManager::CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec,
                                      geometry_msgs::msg::Quaternion &q)
{
  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void VoxelMapManager::mapSliding()
{
  if((position_last_ - last_slide_position).norm() < config_setting_.sliding_thresh)
  {
    LOG(INFO)<<RED<<"[DEBUG]: Last sliding length "<<(position_last_ - last_slide_position).norm()<<RESET<<"\n";
    return;
  }

  //get global id now
  last_slide_position = position_last_;
  double t_sliding_start = omp_get_wtime();
  float loc_xyz[3];
  for (int j = 0; j < 3; j++)
  {
    loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
    if (loc_xyz[j] < 0) { loc_xyz[j] -= 1.0; }
  }
  // VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);//discrete global
  clearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size, (int64_t)loc_xyz[0] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[1] + config_setting_.half_map_size, (int64_t)loc_xyz[1] - config_setting_.half_map_size,
                    (int64_t)loc_xyz[2] + config_setting_.half_map_size, (int64_t)loc_xyz[2] - config_setting_.half_map_size);
  double t_sliding_end = omp_get_wtime();
  LOG(INFO)<<RED<<"[DEBUG]: Map sliding using "<<t_sliding_end - t_sliding_start<<" secs"<<RESET<<"\n";
  return;
}

void VoxelMapManager::clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min )
{
  int delete_voxel_cout = 0;
  // double delete_time = 0;
  // double last_delete_time = 0;
  for (auto it = voxel_map_.begin(); it != voxel_map_.end(); )
  {
    const VOXEL_LOCATION& loc = it->first;
    bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max || loc.y < y_min || loc.z > z_max || loc.z < z_min;
    if (should_remove){
      // last_delete_time = omp_get_wtime();
      delete it->second;
      it = voxel_map_.erase(it);
      // delete_time += omp_get_wtime() - last_delete_time;
      delete_voxel_cout++;
    } else {
      ++it;
    }
  }
  LOG(INFO)<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" root voxels"<<RESET<<"\n";
  // LOG(INFO)<<RED<<"[DEBUG]: Delete "<<delete_voxel_cout<<" voxels using "<<delete_time<<" s"<<RESET<<"\n";
}