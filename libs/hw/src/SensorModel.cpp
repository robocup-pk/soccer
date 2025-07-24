#include "SensorModel.h"

int hw::SensorModel::GetTicks() {
  if (!initialized_sensor) return ticks_discrete;
  UpdateTicks();
  return ticks_discrete;
}

double hw::SensorModel::GetWheelSpeedRadps() {
  double rev_ps = rpm / 60.0;
  return rev_ps * 2 * M_PI;
}

void hw::SensorModel::UpdateTicks() {
  rpm_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt = rpm_end_time - rpm_start_time;
  rpm_start_time = std::chrono::high_resolution_clock::now();

  double rev_ps = rpm / 60.0;
  double num_revs = rev_ps * dt.count();
  double d_ticks = num_revs * hw::Config::ticks_per_rev;
  ticks_continuous += d_ticks;
  ticks_discrete = std::round(ticks_continuous);
}

void hw::SensorModel::SetWheelSpeedRpm(double rpm) {
  if (!initialized_sensor) {
    initialized_sensor = true;
    this->rpm = rpm;
    rpm_start_time = std::chrono::high_resolution_clock::now();
    return;
  }
  UpdateTicks();
  this->rpm = rpm;
}

double hw::SensorModel::GetRpm() { return rpm; }

void hw::SensorModel::Clear() {
  ticks_discrete = 0;
  ticks_continuous = 0;
  rpm = 0;
  initialized_sensor = false;
  w_radps = 0;
}

void hw::SensorModel::SetAngularVelocityRadps(double w_radps) { this->w_radps = w_radps; }

double hw::SensorModel::GetAngularVelocityRadps() { return w_radps; }