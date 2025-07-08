#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <cmath>
#include <chrono>

#include "HwConfig.h"

namespace hw {

class MotorModel {
 public:
  int GetTicks();
  double GetRpm();

  void SetWheelSpeedRpm(double rpm);
  void Clear();

  double GetWheelSpeedRadps();
  void UpdateTicks();

 private:
  double rpm = 0;
  int ticks_discrete = 0;
  double ticks_continuous = 0;  // not rounded off
  std::chrono::time_point<std::chrono::high_resolution_clock> rpm_end_time, rpm_start_time;

  bool initialized_motor = false;
};
}  // namespace hw

#endif  // MOTOR_MODEL_H