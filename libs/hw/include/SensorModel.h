#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

#include <cmath>
#include <chrono>
#include "HwConfig.h"

namespace hw {

class SensorModel {
 public:
  // Motor Model methods
  int GetTicks();
  double GetRpm();

  void SetWheelSpeedRpm(double rpm);
  void Clear();

  double GetWheelSpeedRadps();
  void UpdateTicks();

  // Gyro Model methods
  void SetAngularVelocityRadps(double rpm);
  double GetAngularVelocityRadps();

 private:
  // Motor Model private members
  double rpm = 0;
  int ticks_discrete = 0;
  double ticks_continuous = 0;  // not rounded off
  std::chrono::time_point<std::chrono::high_resolution_clock> rpm_end_time, rpm_start_time;
  bool initialized_sensor = false;

  // Gyro Model private members
  double w_radps = 0;
};

}  // namespace hw

#endif  // SENSOR_MODEL_H