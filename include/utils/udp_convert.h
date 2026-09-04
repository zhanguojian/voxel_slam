#pragma once

#include <gflags/gflags.h>
#include "glog/logging.h"

#include <fcntl.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <ctime>

#include <pcap.h>

const uint32_t ONE_FRAME_POINT_NUM_LIVOX = 20064;    ///< MID360一帧点云数量

#define EPOLL_MAX_EVENTS 10
#define BUFFER_SIZE 2048

#pragma pack(push, 1)

// Ethernet 头（14字节）
struct EthernetHeader {
  uint8_t destination[6];
  uint8_t source[6];
  uint16_t type;  // 0x0800 = IPv4
};

// SLLv2 头（20字节）
struct SLLv2Header {
  uint16_t Protocol;     // 协议类型，0x0800 = IPv4
  uint16_t placeholder;  // Placeholder: 保留字段
  uint32_t ifindex;      // Interface index: 网卡接口索引
  uint16_t arp_type;     // Link-layer address type: 链路层硬件类型
  uint8_t pkt_type;      // Packet type: 报文类型
  uint8_t addr_len;      // Link-layer address length: 链路地址长度
  uint8_t source[6];     // Source: 源MAC地址
  uint16_t unused;       // Unused: 保留字段
};

// IP 头（最小20字节）
struct IPHeader {
  uint8_t version_header_length;  // 高4位 version，低4位 header length
  uint8_t type_of_service;
  uint16_t total_length;
  uint16_t identification;
  uint16_t flags_offset;
  uint8_t time_to_live;
  uint8_t protocol;  // 17 = UDP
  uint16_t header_checksum;
  uint32_t source_address;
  uint32_t destination_address;
};

// UDP 头（标准8字节）
struct UDPHeader {
  uint16_t source_port;
  uint16_t destination_port;
  uint16_t length;
  uint16_t checksum;
};

// IMU数据结构
struct ImuData {
  float gyro_x;  // rad/s
  float gyro_y;
  float gyro_z;
  float acc_x;  // g
  float acc_y;
  float acc_z;
};

// 点云数据结构
struct PointCloudData {
  int32_t x;  // mm
  int32_t y;
  int32_t z;
  uint8_t intensity;
  uint8_t label;
};

struct PointCloudXYZIRC {
  float x = 0.;
  float y = 0.;
  float z = 0.;
  float intensity = 0.;
  double curvature = 0.;
};

// 点云和IMU数据的协议头
struct LivoxHeader {
  uint8_t version;         // 协议版本，一字节
  uint16_t length;         // UDP数据长度，两字节
  uint16_t time_interval;  // 采样时间，两字节 单位：0.1us
  uint16_t dot_num;        // data字段中的数量，两字节
  uint16_t udp_cnt;        // UDP包计数，两字节
  uint8_t frame_cnt;       // 点云帧计数，一字节
  uint8_t data_type;       // 数据类型，一字节, 0:IMU数据，1:单回波模式直角坐标系点云数据
  uint8_t time_type;       // 时间戳类型，一字节,
                           // 0:无同步源，时间戳为雷达开机时间，1:gPTP/PTP同步，时间戳为master时钟源时间，2:GPS时间同步
  uint8_t reserved[12];    // 保留字段，12字节
  uint32_t crc32;          // CRC32校验码，4字节
  uint64_t timestamp;      // 时间戳，8字节 单位：ns
};

#pragma pack(pop)

enum class PacketType {
  IMU = 0,    ///< IMU消息
  LIDAR,      ///< 点云消息(不满一帧)
  LIDARFULL,  ///< 点云消息(一帧点云)
  ERROR,      ///< 其他消息
};


struct Packet {
  uint8_t data[BUFFER_SIZE];       ///< 原始数据
  int32_t len;                     ///< 原始数据实际长度
  struct sockaddr_in client_addr;  ///< 源IP
  int32_t source_port;             ///< 源端口
  uint64_t rcv_timestamp;          ///< 当前packet到达网卡时间
};

struct PacketInfo {
  // 网络层
  size_t l2_len = 0;        ///< L2层数据长度
  uint16_t src_port = 0;    ///< 源端口
  uint16_t dst_port = 0;    ///< 目标端口
  uint16_t udp_length = 0;  ///< UDP数据长度

  // Livox
  const LivoxHeader* livox_header = nullptr;  ///< Livox头
  const uint8_t* livox_data = nullptr;        ///< Livox数据
};


/**
 * @brief 循环数组，用于接受网卡数据
 *
 */
class PacketRingBuffer {
 public:

  Packet* acquireWriteSlot() {
    const size_t head = head_;
    const size_t next = increment(head);

    if (next == tail_) {
      return nullptr;
    }

    return &ring_[head];
  }


  void commitWrite() { head_ = increment(head_); }

  Packet* acquireReadSlot() {
    if (tail_ == head_) {
      return nullptr;
    }

    return &ring_[tail_];
  }


  void commitRead() { tail_ = increment(tail_); }


  bool empty() {
    if (tail_ == head_) {
      return true;
    } else {
      return false;
    }
  }

 private:

  size_t increment(size_t idx) { return (idx + 1) % kRingSize; }

 private:
  static constexpr size_t kRingSize = 4096;

  Packet ring_[kRingSize];

  std::atomic<size_t> head_{0};
  std::atomic<size_t> tail_{0};
};


// 添加O_NONBLOCK标志
int setNonBlocking(int fd) {
  // 1. 获取当前状态
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    return -1;
  }
  // 2. 添加O_NONBLOCK标志
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}


int createUdpSocket(const std::string& ip, int port) {

  // 0. 创建socket
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    LOG(ERROR) << "socket creation failed for " << ip << ":" << port;
    return -1;
  }

// 1. 初始化socket，设置地址复用、接收时间戳
  int opt = 1;
  if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    LOG(ERROR) << "setsockopt SO_REUSEADDR failed for " << ip << ":" << port;
    close(sock_fd);
    return -1;
  }
  if (setsockopt(sock_fd, SOL_SOCKET, SO_TIMESTAMPNS, &opt, sizeof(opt)) < 0) {
    LOG(ERROR) << "setsockopt SO_TIMESTAMPNS failed for " << ip << ":" << port;
    close(sock_fd);
    return -1;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);

  // 2. 将字符串IP转换为网络地址
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
    LOG(ERROR) << "invalid ip address: " << ip;
    close(sock_fd);
    return -1;
  }

    // 3. 绑定IP和端口
  if (bind(sock_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    LOG(ERROR) << "bind failed for " << ip << ":" << port;
    close(sock_fd);
    return -1;
  }

    // 4. 设置为非阻塞模式
  if (setNonBlocking(sock_fd) < 0) {
    LOG(ERROR) << "setNonBlocking failed for " << ip << ":" << port;
    close(sock_fd);
    return -1;
  }

    LOG(INFO) << "UDP socket created and bound to " << ip << ":" << port;

  return sock_fd;
}

int addToEpoll(int epoll_fd, int sock_fd) {
  struct epoll_event ev;
  ev.events = EPOLLIN;  // 使用 LT 模式
  ev.data.fd = sock_fd;
  if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sock_fd, &ev) < 0) {
    LOG(ERROR) << "epoll_ctl add failed for fd " << sock_fd;
    return -1;
  }
  return 0;
}

uint64_t getUnixTimestamp() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}


//从 UDP socket 接收一包数据，同时读取 Linux 内核给这包 UDP 数据附加的纳秒级接收时间戳，
// 并把时间戳转换成 uint64_t 纳秒返回，定位使用检测解析数据是否接受延迟
uint64_t recvUdpPacketWithTimestamp(int fd, Packet* packet) {

  // 1. 接受带时间戳的数据
  char control[1024];

  //收到的普通 UDP 数据应该放到哪里。
  struct iovec iov;
  //1. 设置一个udp包的缓存，把UDP内容放到 packet->data
  iov.iov_base = packet->data;
  //2. 设置缓冲区大小,把收到多少字节放到 packet->len
  iov.iov_len = BUFFER_SIZE;

  //UDP 接收配置包。
  struct msghdr msg{};

  //把发送这个 UDP 包的设备地址放到 client_addr。
  msg.msg_name = &packet->client_addr;
  msg.msg_namelen = sizeof(packet->client_addr);
  
  //UDP 的 payload 放到 iov 描述的内存。
  msg.msg_iov = &iov;
  //实际上 iovec 可以有多个，但这里不需要，所以只有一个。
  msg.msg_iovlen = 1;

  //把 socket 的辅助信息全部写进 control。
  msg.msg_control = control;
  msg.msg_controllen = sizeof(control);

  //接收 UDP 数据包，返回实际接收到的字节数
  packet->len = static_cast<int32_t>(recvmsg(fd, &msg, 0));

  // 2. 解析Packet接受时间戳
  timespec kernel_ts{};
  for (cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS) {
      memcpy(&kernel_ts, CMSG_DATA(cmsg), sizeof(kernel_ts));
    }
  }

  packet->rcv_timestamp = (kernel_ts.tv_sec * 1000000000ULL + kernel_ts.tv_nsec);

  return packet->rcv_timestamp;
}