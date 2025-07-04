#ifndef GYRO_MODEL_H
#define GYRO_MODEL_H

#include <cmath>
#include <chrono>

namespace hw {

class GyroModel {
 public:
  void SetAngularVelocityRadps(double rpm);
  void Clear();
  double GetAngularVelocityRadps();

 private:
  double w_radps = 0;
};
}  // namespace hw

#endif  // GYRO_MODEL_H