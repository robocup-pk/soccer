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
  HardwareManager(size_t numMotors = 4) : motor_driver_(numMotors) {}
  ~HardwareManager() { motor_driver_.stop(); }

  void init(const std::string& port) { motor_driver_.init(port); }
  void setSpeeds(const std::vector<int>& speeds) {}

 private:
  MotorDriver motor_driver_;
  std::mutex init_mutex_;
};
}  // namespace hw

#endif  // HARDWARE_MANAGER_H