#pragma once

#include "UdpReceiver.h"
#include "vision/ssl_vision_wrapper.pb.h"
#include <vector>
#include <mutex>
#include <atomic>

namespace comm {

// data for one robot
struct RobotVision {
  uint32_t id;
  float x, y, orientation;
};

// Receives SSL_WrapperPacket over UDP, keeps two vectors of RobotVision
class VisionManager {
 public:
  // Multicast addr + port (defaults match SSL-Vision)
  VisionManager(const std::string& multicast_addr = "224.5.23.2", int port = 10006);
  ~VisionManager();

  // Manual control
  bool start();  // returns false if receiver fails to start
  void stop();

  // Thread-safe accessors
  std::vector<RobotVision> getYellowRobots();
  std::vector<RobotVision> getBlueRobots();

 private:
  void onPacket(const SSL_WrapperPacket& pkt);

  UdpReceiver receiver_;
  std::atomic<bool> running_;
  std::mutex mutex_;
  std::vector<RobotVision> yellow_;
  std::vector<RobotVision> blue_;
};

}  // namespace comm