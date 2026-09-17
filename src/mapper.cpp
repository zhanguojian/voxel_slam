#include "mapper.h"
#include "io.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <tuple>
#include "ui/pangolin_window.h"

Mapper::Mapper(const RunConfig& config) : config_(config),imu_(config),map_(config.voxel) {
  map_.extR_=config.extR;
  map_.extT_=config.extT;
}

bool Mapper::sync_packages(MeasureGroup &meas)
{

  std::unique_lock<std::mutex> lock(mtx_buffer);

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
        meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / 1000.0;

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

    meas.measures.push_back(m);
    lidar_pushed = false;

    sig_buffer.notify_all();
    return true;

}

void Mapper::process(const MeasureGroup& measures) 
{
  auto cloud=PointCloudXYZI::Ptr(new PointCloudXYZI);

  if(!imu_.Process(measures,state_,*cloud)) 
  { 
    ++skipped_; return; 
  };

  auto down=PointCloudXYZI::Ptr(new PointCloudXYZI);

  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(config_.filter_size,config_.filter_size,config_.filter_size);

  filter.setInputCloud(cloud); filter.filter(*down);

  if(down->size()<6) 
  { 
    ++skipped_; return; 
   };

  map_.state_=state_; 
  map_.feats_undistort_=cloud; 
  map_.feats_down_body_=down;

  if(!map_initialized_) 
  {
    map_.BuildVoxelMap();
    map_initialized_=true;

  } else 
  {
    if(!map_.StateEstimation(state_)) 
    {
      // 匹配失败只保留 IMU 预测，不把未约束点云混入地图。
      ++unmatched_;
      LOG(WARNING)<<"Frame rejected by point-to-plane matching at "<<std::setprecision(16)<<measures.lidar.end;
      return;
    }

    state_=map_.state_;

    map_.UpdateVoxelMap(map_.pv_list_);
  };


  map_.position_last_=state_.pos_end;

  if(config_.voxel.map_sliding_en)
  {
    map_.mapSliding();
  };

  auto world=PointCloudXYZI::Ptr(new PointCloudXYZI);

  world->reserve(down->size());

  for(const auto& p:*down) 
  {
    const V3D w = state_.rot_end * ( config_.extR * V3D(p.x,p.y,p.z) + config_.extT) + state_.pos_end;

    if(!w.allFinite() || w.cwiseAbs().maxCoeff()>1e8) t
    {
        hrow std::runtime_error("Invalid world point or divergent pose");
    }

    PointType q=p; q.x=w.x(); q.y=w.y(); q.z=w.z(); world->push_back(q);

    const VOXEL_LOCATION key(static_cast<int64_t>(std::floor(w.x()/config_.save_voxel_size)),
        static_cast<int64_t>(std::floor(w.y()/config_.save_voxel_size)),
        static_cast<int64_t>(std::floor(w.z()/config_.save_voxel_size)));

    auto& cell=output_map_[key]; cell.sum+=w; cell.intensity+=p.intensity; ++cell.count;
  };

  const Eigen::Quaterniond q(state_.rot_end);

  trajectory_<<std::fixed<<std::setprecision(9)<<measures.lidar.end<<" "
    <<state_.pos_end.transpose()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";

  if(!trajectory_) 
  {
    throw std::runtime_error("Failed to write trajectory");
  };


  if(config_.save_scans) {
    const auto path=run_directory_/"scan_frame"/(std::to_string(processed_)+".pcd");
    if(pcl::io::savePCDFileBinary(path.string(),*cloud)<0) throw std::runtime_error("Failed to save scan");
  };

  if(display_) 
  {
    display_(world,state_);
  };

  ++processed_;
  LOG_EVERY_N(INFO,50)<<"Mapped frames="<<processed_<<", active voxels="<<map_.voxel_map_.size()
                     <<", output voxels="<<output_map_.size();
}



// void Mapper::run()
// {
  
//     std::atomic<bool> pcap_finished{false};


//     PcapIO pcap(pcap_file);


//     pcap.addPointCloud2Handle(
//         lid_topic,
//         [this](PointCloudMsgPtr m) {

//         mtx_buffer.lock();

//         double cur_head_time = m->header.stamp.toSec();

//         if (cur_head_time < last_timestamp_lidar)
//         {
//             LOG(ERROR) << "lidar loop back, clear buffer";
//             lid_raw_data_buffer.clear();
//         }
//         lid_raw_data_buffer_.push_back(m);
//         lid_header_time_buffer.push_back(cur_head_time);
//         lid_header_time_buffer.push_back(cur_head_time);
//         last_timestamp_lidar = cur_head_time;
//         mtx_buffer.unlock();
//         sig_buffer.notify_all();
//         })
//     .addIMUHandle(
//         imu_topic,
//         [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {

//         double timestamp = m->header.stamp.toSec();

//         if (fabs(last_timestamp_lidar - timestamp) > 0.5 )
//         {
//             LOG(WARNING) << "IMU and LiDAR not synced! delta time: " << last_timestamp_lidar - timestamp;
//         };

//         mtx_buffer.lock();

//         if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
//         {
//             mtx_buffer.unlock();
//             sig_buffer.notify_all();
//             LOG(ERROR) << "imu loop back, offset: " << last_timestamp_imu - timestamp;
//             return;
//         }

//         last_timestamp_imu = timestamp;
//         imu_buffer.push_back(m);
//         // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
//         mtx_buffer.unlock();
//         if (imu_prop_enable)
//         {
//             mtx_buffer_imu_prop.lock();
//             if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*m); }
//             newest_imu = *m;
//             new_imu = true;
//             mtx_buffer_imu_prop.unlock();
//         }
//         sig_buffer.notify_all();
//         })
//     .go();

//     pcap_finished.store(true, std::memory_order_release);
//     LOG(INFO) << "PCAP LiDAR/IMU 读取完成";
//     sig_buffer.notify_all();

//     rclcpp::Rate rate(5000);

//     while (rclcpp::ok())
//     {
//       if (!sync_packages(Measures))
//       {
//         const bool pcap_done =
//           pcap_finished.load(std::memory_order_acquire);


//         if (pcap_done)
//         {
//           LOG(INFO)<<"PCAP/RAW producer 已全部结束，当前 buffer 无法继续组成测量组，退出离线处理";
//           break;
//         }

//         rate.sleep();
//         continue;
//       }

//       handleFirstFrame();
//       processImu();
//       handlelio();
//     }

//     if (pcap_thread.joinable())
//     {
//       pcap_thread.join();
//     }


//     savePCD();
//     return;
// }

void Mapper::run() 
{

  std::filesystem::create_directories(config_.output_path);

  const auto id=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  run_directory_=config_.output_path/("run_"+std::to_string(id));

  if(!std::filesystem::create_directory(run_directory_))
  {
    LOG(ERROR)<<"Output run directory already exists";
  };

  if(config_.save_scans) 
  {
    std::filesystem::create_directory(run_directory_/"scan_frame");
  };

  trajectory_.open(run_directory_/"trajectory.txt");

  if(!trajectory_) 
  {
    throw std::runtime_error("Cannot open trajectory output");
  };

  trajectory_<<"# timestamp tx ty tz qx qy qz qw (world_T_imu)\n";

  std::unique_ptr<PangolinWindow> viewer;

  if(config_.viewer)
  {
    viewer=std::make_unique<PangolinWindow>(config_.viewer_scans);
    display_=[&](PointCloudXYZI::ConstPtr cloud,const StatesGroup& s){ viewer->update(cloud,s); };
  };


    PcapIO pcap(pcap_file);


    pcap.addPointCloud2Handle(
        lid_topic,
        [this](PointCloudMsgPtr m) {

        mtx_buffer.lock();

        double cur_head_time = m->header.stamp.toSec();

        if (cur_head_time < last_timestamp_lidar)
        {
            LOG(ERROR) << "lidar loop back, clear buffer";
            lid_raw_data_buffer.clear();
        }
        lid_raw_data_buffer_.push_back(m);
        lid_header_time_buffer.push_back(cur_head_time);
        lid_header_time_buffer.push_back(cur_head_time);
        last_timestamp_lidar = cur_head_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        })
    .addIMUHandle(
        imu_topic,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr &m) {

        double timestamp = m->header.stamp.toSec();

        if (fabs(last_timestamp_lidar - timestamp) > 0.5 )
        {
            LOG(WARNING) << "IMU and LiDAR not synced! delta time: " << last_timestamp_lidar - timestamp;
        };

        mtx_buffer.lock();

        if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
        {
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            LOG(ERROR) << "imu loop back, offset: " << last_timestamp_imu - timestamp;
            return;
        }

        last_timestamp_imu = timestamp;
        imu_buffer.push_back(m);
        // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
        mtx_buffer.unlock();
        if (imu_prop_enable)
        {
            mtx_buffer_imu_prop.lock();
            if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*m); }
            newest_imu = *m;
            new_imu = true;
            mtx_buffer_imu_prop.unlock();
        }
        sig_buffer.notify_all();
        })
    .go();

  try {
    if(!imu_.initialized()) 
    {
        LOG(error)<<"Capture ended before IMU initialization completed";
    };
    if(!processed_) 
     {
          LOG(error)<< "No mapped frame; check point filtering, frame times and capture duration";
     }
     
    saveMap();
    
    std::ofstream summary(run_directory_/"summary.txt");
    summary<<"source="<<config_.source_path<<"\nprocessed="<<processed_<<"\nskipped="<<skipped_
           <<"\nunmatched="<<unmatched_<<"\nmap_points="<<output_map_.size()<<"\n";
    if(!summary) throw std::runtime_error("Cannot write summary");
  } catch(...) {
    // 不把错误中断伪装成成功的完整地图。
    std::ofstream(run_directory_/"INCOMPLETE.txt")<<"Run failed; trajectory may be partial. See terminal log.\n";
    display_={};
    throw;
  }
  display_={};
  LOG(INFO)<<"Finished: mapped="<<processed_<<", skipped="<<skipped_<<", unmatched="<<unmatched_;
}


void Mapper::saveMap() {
  if(output_map_.empty()) return;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(output_map_.size());
  // 排序输出，便于回归比较与定位具体体素。
  std::vector<VOXEL_LOCATION> keys;
  keys.reserve(output_map_.size());
  for(const auto& item:output_map_) keys.push_back(item.first);
  std::sort(keys.begin(),keys.end(),[](const auto& a,const auto& b){
    return std::tie(a.x,a.y,a.z)<std::tie(b.x,b.y,b.z);
  });
  for(const auto& key:keys) {
    const auto& cell=output_map_.at(key);
    const V3D p=cell.sum/double(cell.count);
    pcl::PointXYZI q; q.x=p.x(); q.y=p.y(); q.z=p.z(); q.intensity=cell.intensity/cell.count;
    cloud.push_back(q);
  }
  if(pcl::io::savePCDFileBinary((run_directory_/"map.pcd").string(),cloud)<0)
    throw std::runtime_error("Failed to write map.pcd");
  trajectory_.flush();
  if(!trajectory_) throw std::runtime_error("Failed to flush trajectory");
  LOG(INFO)<<"Output: "<<run_directory_;
}
