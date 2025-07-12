#include "VisionManager.h"
#include <stdexcept>

namespace comm {

VisionManager::VisionManager(const std::string& addr, int port)
    : receiver_(addr, port),
      running_(false)  // start stopped
{
  receiver_.setProtobufCallback<SSL_WrapperPacket>(
      [this](const SSL_WrapperPacket& pkt) { onPacket(pkt); });
  // start receiving immediately
  if (!start()) {
    throw std::runtime_error("VisionManager: failed to start UDPReceiver");
  }
}

VisionManager::~VisionManager() { stop(); }

bool VisionManager::start() {
  if (running_) {
    return true;  // already running
  }
  running_ = true;
  if (!receiver_.start()) {
    running_ = false;
    return false;
  }
  return true;
}

void VisionManager::stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  receiver_.stop();
}

void VisionManager::onPacket(const SSL_WrapperPacket& pkt) {
  if (!running_) return;
  std::lock_guard<std::mutex> lk(mutex_);
  yellow_.clear();
  blue_.clear();
  if (pkt.has_detection()) {
    for (const auto& r : pkt.detection().robots_yellow()) {
      yellow_.push_back({r.robot_id(), r.x(), r.y(), r.orientation()});
    }
    for (const auto& r : pkt.detection().robots_blue()) {
      blue_.push_back({r.robot_id(), r.x(), r.y(), r.orientation()});
    }
  }
}

std::vector<RobotVision> VisionManager::getYellowRobots() {
  std::lock_guard<std::mutex> lk(mutex_);
  return yellow_;
}

std::vector<RobotVision> VisionManager::getBlueRobots() {
  std::lock_guard<std::mutex> lk(mutex_);
  return blue_;
}

}  // namespace comm