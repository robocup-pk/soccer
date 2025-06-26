#ifndef HW_MOTOR_DRIVER_H
#define HW_MOTOR_DRIVER_H
#include <libserial/SerialPort.h>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <chrono>

namespace hw {

class MotorDriver {
 public:
  MotorDriver(size_t numMotors);
  ~MotorDriver();

  void init(const std::string& port = "/dev/ttyAMA0");
  void setSpeeds(const std::vector<int>& speeds);
  void start();
  void stop();

 private:
  void controlLoop();
  void senseLoop();

  size_t num_motors_;
  LibSerial::SerialPort serial_port_;
  std::vector<int> desired_speeds_;
  std::vector<int> sensed_speeds_;
  std::atomic<bool> running_;
  std::mutex speed_mutex_;

  std::thread control_thread_;
  std::thread sense_thread_;
};

}  // namespace hw

#endif HW_MOTOR_DRIVER_H