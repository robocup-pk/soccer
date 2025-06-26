#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H

#include "MotorDriver.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <string>

namespace hw {
class HardwareManager {
 public:
  HardwareManager(size_t numMotors = 4) : motor_driver_(numMotors) { init(); }

  ~HardwareManager() { motor_driver_.stop(); }

  void init(const std::string& port = "/dev/ttyAMA0") { motor_driver_.init(port); motor_driver_.start(); }
  void setSpeeds(const std::vector<int>& speeds) {}

 private:
  MotorDriver motor_driver_;
  std::mutex init_mutex_;
};
}  // namespace hw

#endif  // HARDWARE_MANAGER_H