#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <cmath>
#include <chrono>

#include "HwConfig.h"

namespace hw {

class MotorModel {
 public:
  int GetTicks() {
    if (!initialized_motor) return ticks;
    UpdateTicks();

    return ticks;
  }

  double GetWheelSpeedRadps() {
    double rev_ps = rpm / 60.0;
    return rev_ps * 2 * M_PI;
  }

  void UpdateTicks() {
    end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt = end_time - start_time;
    start_time = std::chrono::high_resolution_clock::now();

    double rev_ps = rpm / 60.0;
    double num_revs = rev_ps * dt.count();

    double d_ticks = num_revs * hw::Config::ticks_per_rev;

    ticks += d_ticks;
  }

  void SetWheelSpeedRpm(double rpm) {
    if (!initialized_motor) {
      initialized_motor = true;
      this->rpm = rpm;
      start_time = std::chrono::high_resolution_clock::now();
      return;
    }
    UpdateTicks();
    this->rpm = rpm;
    start_time = std::chrono::high_resolution_clock::now();
  }

  void Clear() {
    ticks = 0;
    rpm = 0;
    initialized_motor = false;
  }

 private:
  double rpm = 0;
  int ticks = 0;
  std::chrono::time_point<std::chrono::high_resolution_clock> end_time, start_time;

  bool initialized_motor = false;
};
}  // namespace hw

#endif  // MOTOR_MODEL_H