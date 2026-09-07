
#pragma once

#include <functional>
#include "msg_type.h"
#include "udp_convert.h"


class DataIO {
 public:
  explicit DataIO(const std::string &filename) { data_filen_path_ = filename; }

  virtual ~DataIO() = default;

  using MessageProcessFunction = std::function<bool(const rosbag2_storage::SerializedBagMessageSharedPtr m)>;

  using PointCloud2Handle = std::function<bool(PointCloud2MsgPtr)>;

  using ImuHandle = std::function<bool(IMUPtr)>;



  virtual void go() = 0;

  DataIO &addHandle(const std::string &topic_name, MessageProcessFunction func) {
    process_func_.emplace(topic_name, func);
    return *this;
  }


  virtual DataIO &addPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) = 0;


  virtual DataIO &addIMUHandle(const std::string &topic_name, ImuHandle f) = 0;

  void cleanProcessFunc() { process_func_.clear(); }

 protected:

  std::map<std::string, MessageProcessFunction> process_func_;

  std::string data_filen_path_;
};


class PcapIO : public DataIO 
{
 public:
  explicit PcapIO(const std::string &filename, const int32_t &pointcloud_port, const int32_t &imu_port)
      : filename_(filename), pointcloud_port_(pointcloud_port), imu_port_(imu_port)
  {
    // 注意，imu消息不需要构造消息头，只对点云消息头处理，用于持续使用
    lidar_msg_ = std::make_shared<PointCloudMsg>();
    lidar_msg_->fields.clear();
    lidar_msg_->fields.reserve(7);
    lidar_msg_->is_dense = true;
    lidar_msg_->height = 1;
    int offset = 0;
    offset = addPointField(*lidar_msg_, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(*lidar_msg_, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(*lidar_msg_, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(*lidar_msg_, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(*lidar_msg_, "curvature", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
    lidar_msg_->point_step = offset;
    lidar_msg_->header.frame_id = "livox_frame";
    lidar_msg_->width = ONE_FRAME_POINT_NUM_LIVOX;
    lidar_msg_->row_step = lidar_msg_->point_step * lidar_msg_->width;
    lidar_msg_->data.resize(lidar_msg_->row_step);
  }


    //检查packet是否合法
    int32_t checkPacket(const pcap_pkthdr *header, const uint8_t *data, PacketInfo &info);

    //处理每个packet
    PacketType parsePacket(const PacketInfo &info);

    void go() override;

        DataIO &addPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) override;
        DataIO &addIMUHandle(const std::string &topic_name, ImuHandle f) override;

    private:

        std::string filename_;
        int32_t pointcloud_port_;
        int32_t imu_port_;


        IMUPtr imu_msg_;               ///< IMU消息
        PointCloud2MsgPtr lidar_msg_;  ///< 点云消息


        uint16_t last_imu_cnt_ = 0;    ///< imu packet计数
        uint16_t last_lidar_cnt_ = 0;  ///< lidar packet计数
        uint32_t cur_point_num_ = 0;   ///< 当前点云累积点数量

}



int32_t PcapIO::checkPacket(const pcap_pkthdr* header, const uint8_t* data, PacketInfo& info) 
{
  info = PacketInfo();

  const uint8_t* l2_data = data;

  // 自动判断 SLLv2 / 以太网
  if (header->caplen >= sizeof(SLLv2Header)) {
    const SLLv2Header* sll2 = reinterpret_cast<const SLLv2Header*>(l2_data);
    if (ntohs(sll2->Protocol) == 0x0800) {
      info.l2_len = sizeof(SLLv2Header);
    }
  }

  // 不是SLLv2，则按标准以太网解析
  if (info.l2_len == 0) {
    if (header->caplen < sizeof(EthernetHeader)) {
      // LOG(ERROR) << "以太网数据包太短";
      return -1;
    }
    const EthernetHeader* eth = reinterpret_cast<const EthernetHeader*>(l2_data);
    if (ntohs(eth->type) != 0x0800) {
      // LOG(ERROR) << "不是IPv4数据包";
      return -1;
    }
    info.l2_len = sizeof(EthernetHeader);
  }

  // IP头检查
  if (header->caplen < info.l2_len + sizeof(IPHeader)) {
    // LOG(ERROR) << "IP数据包太短";
    return -1;
  }

  // 计算IP头实际长度
  const IPHeader* ip = reinterpret_cast<const IPHeader*>(l2_data + info.l2_len);
  int ip_header_len = (ip->version_header_length & 0x0F) * 4;
  if (ip_header_len < 20) {
    // LOG(ERROR) << "IP头部长度不合法";
    return -1;
  }

  // 只处理 UDP
  if (ip->protocol != 17) {
    // LOG(ERROR) << "不是UDP数据包";
    return -1;
  }

  // UDP头检查
  const uint8_t* udp_ptr = reinterpret_cast<const uint8_t*>(ip) + ip_header_len;
  if ((udp_ptr + sizeof(UDPHeader)) > (data + header->caplen)) {
    // LOG(ERROR) << "UDP数据包太短";
    return -1;
  }

  // 端口过滤
  const UDPHeader* udp = reinterpret_cast<const UDPHeader*>(udp_ptr);

  info.src_port = ntohs(udp->source_port);
  info.dst_port = ntohs(udp->destination_port);
  info.udp_length = ntohs(udp->length);

  if (info.dst_port != pointcloud_port_ && info.dst_port != imu_port_) {
    // LOG(ERROR) << "未知端口";
    return -1;
  }

  // UDP长度合法性检查
  if (static_cast<size_t>(info.udp_length) < sizeof(UDPHeader)) {
    // LOG(ERROR) << "UDP长度不合法";
    return -1;
  }

  // payload边界检查
  const uint8_t* payload = udp_ptr + sizeof(UDPHeader);
  int payload_len = static_cast<int>(info.udp_length - sizeof(UDPHeader));
  if (payload + payload_len > data + header->caplen) {
    // LOG(ERROR) << "Payload超出数据包边界";
    return -1;
  }

  // 填充LIVOX数据包
  const LivoxHeader* livox =
      reinterpret_cast<const LivoxHeader*>(reinterpret_cast<const uint8_t*>(udp) + sizeof(UDPHeader));

  if ((uint8_t*)livox + sizeof(LivoxHeader) > data + header->caplen) {
    return -1;
  }

  info.livox_header = livox;
  info.livox_data = reinterpret_cast<const uint8_t*>(livox) + sizeof(LivoxHeader);

  return 0;
}



PacketType PcapIO::parsePacket(const PacketInfo& info) {
  // 0. 直接偏移到实际数据域获取所需数据
  uint16_t dst_port = info.dst_port;
  uint16_t payload_length = info.udp_length - sizeof(UDPHeader);
  const LivoxHeader* livox_header = info.livox_header;
  const uint8_t* livox_data = info.livox_data;

  // 1. 检测当前packet是否完整
  int livox_len = livox_header->length;
  if (livox_len <= 0 || livox_len > payload_length) {
    LOG(ERROR) << "数据存在缺失，丢弃该数据包";
    return PacketType::ERROR;
  }

  // LOG(INFO) << "Livox Packet | Port: " << dst_port << " | DataType: " << static_cast<int>(livox_header->data_type)
  //           << " | Timestamp: " << livox_header->timestamp << " | Total Packets: " << livox_header->udp_cnt;

  // 2. 通过端口号来区分IMU和点云数据
  if (dst_port == imu_port_ && livox_header->data_type == 0) {
    // 2.1.1 校验当前packet激光点是否完整
    if (static_cast<size_t>(livox_len) < sizeof(LivoxHeader) + sizeof(ImuData)) {
      return PacketType::ERROR;
    }

    // 2.1.2 校验是否存在udp包丢失
    if (last_imu_cnt_ && last_imu_cnt_ + 1 != livox_header->udp_cnt) {
      LOG(WARNING) << "UDP包计数器不匹配: " << last_imu_cnt_ << " | 包内计数: " << livox_header->udp_cnt;
    }
    last_imu_cnt_ = livox_header->udp_cnt;

    // 2.1.3 通过内存重映射，直接读取IMU消息
    const ImuData* imu = reinterpret_cast<const ImuData*>(livox_data);
    imu_msg_ =
        std::make_shared<IMU>(static_cast<double>(livox_header->timestamp) * 1e-9,
                              Vec3d(imu->gyro_x, imu->gyro_y, imu->gyro_z), Vec3d(imu->acc_x, imu->acc_y, imu->acc_z));

    return PacketType::IMU;

  } else if (dst_port == pointcloud_port_ && livox_header->data_type == 1) {
    // 2.2.1 校验当前packet激光点是否完整
    int max_points = static_cast<int>((livox_len - sizeof(LivoxHeader)) / sizeof(PointCloudData));
    if (livox_header->dot_num <= 0 || livox_header->dot_num > max_points) {
      LOG(ERROR) << "点云数量不合法: " << livox_header->dot_num << " | 最大点数: " << max_points;
      return PacketType::ERROR;
    }

    // 2.2.2 校验是否存在udp包丢失
    if (last_lidar_cnt_ && static_cast<uint16_t>(last_lidar_cnt_ + 1) != livox_header->udp_cnt) {
      LOG(WARNING) << "UDP包计数器不匹配: " << last_lidar_cnt_ << " | 包内计数: " << livox_header->udp_cnt;
    }
    last_lidar_cnt_ = livox_header->udp_cnt;

    // 2.2.3 计算当前packet中激光点时间增量
    int64_t base_ts = static_cast<int64_t>(livox_header->timestamp);
    double dt = static_cast<double>(livox_header->time_interval) * 100 / livox_header->dot_num;

    // 2.2.4 通过内存重映射，直接对点云消息内存进行写入
    auto* points = reinterpret_cast<PointCloudXYZIRC*>(lidar_msg_->data.data());
    for (int32_t i = 0; i < livox_header->dot_num; ++i) {
      const auto* pt = reinterpret_cast<const PointCloudData*>(livox_data + i * sizeof(PointCloudData));

      auto& p = points[cur_point_num_ + i];
      p.x = static_cast<float>(pt->x) * 1e-3f;
      p.y = static_cast<float>(pt->y) * 1e-3f;
      p.z = static_cast<float>(pt->z) * 1e-3f;
      p.intensity = pt->intensity;
      p.curvature = static_cast<double>(static_cast<uint64_t>(i * dt));
    }
    cur_point_num_ += livox_header->dot_num;

    // 2.2.5 判断是否满一帧（时间戳判定）
    if ((points[cur_point_num_ - 1].timestamp - points[0].timestamp) < 1e8) {
      return PacketType::LIDAR;
    } else {
      lidar_msg_->header.stamp = rclcpp::Time(static_cast<int64_t>(points[0].timestamp));
      cur_point_num_ = 0;
      return PacketType::LIDARFULL;
    }

  } else {
    LOG(ERROR) << "未知数据类型";
  }

  return PacketType::ERROR;
}


void PcapIO::go() {
  LOG(INFO) << "正在使用：" << data_filen_path_;

  // pcap读取
  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_t* handle = pcap_open_offline(data_filen_path_.c_str(), errbuf);

  if (!handle) {
    LOG(ERROR) << "pcap包文件 " << data_filen_path_ << " 无法打开";
    return;
  }

  pcap_pkthdr* header;
  const u_char* data;
  PacketInfo info;

  // 循环读取数据包
  while (pcap_next_ex(handle, &header, &data) >= 0) {
    if (0 != checkPacket(header, data, info)) {
      continue;
    };
    PacketType res = parsePacket(info);

    switch (res) {
      case PacketType::IMU:
            process_func_["/livox/imu"](nullptr);

        break;
      case PacketType::LIDARFULL:
            process_func_["/livox/lidar"](nullptr);
        break;

      default:
        LOG(ERROR) << "未知数据类型";
        break;
    }
  }

  // 关闭pcap文件
  pcap_close(handle);
}

DataIO& PcapIO::addPointCloud2Handle(const std::string&, PointCloud2Handle f) {
  return addHandle("/livox/lidar", [&f, this](const auto&) -> bool { return f(lidar_msg_); });
}

DataIO& PcapIO::addIMUHandle(const std::string&, ImuHandle f) {
  return addHandle("/livox/imu", [&f, this](const auto&) -> bool { return f(imu_msg_); });
}