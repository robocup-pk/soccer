#ifndef GYRO_DRIVER_H
#define GYRO_DRIVER_H

#include <mutex>

#include <libserial/SerialPort.h>

#include "GyroModel.h"

namespace hw {

enum class GyroType { MODEL, REAL };

class GyroDriver {
 public:
  GyroDriver(std::shared_ptr<LibSerial::SerialPort> shared_serial_port);

  void SetAngularVelocityRadps(double w_radps);
  double GetAngularVelocityRadps();
  bool NewDataAvailable();

  bool new_data_available = true;

 private:
#ifdef BUILD_ON_PI
  GyroType gyro_type = GyroType::REAL;
#else
  GyroType gyro_type = GyroType::MODEL;
#endif

  GyroModel gyro;
  double gyro_wradps;

  std::shared_ptr<LibSerial::SerialPort> shared_serial_port;
  std::mutex shared_serial_port_mutex;
};

}  // namespace hw

#endif  // GYRO_DRIVER_H