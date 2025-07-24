#ifndef HW_CONFIG_H
#define HW_CONFIG_H

namespace hw {
struct Config {
  static constexpr int ticks_per_rev = 242;
  static constexpr double gyro_sensitivity = 14.375f; // specific for gyro, hardware dependent
};
}  // namespace hw

#endif  // HW_CONFIG_H