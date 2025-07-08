#include "MotorModel.h"

int hw::MotorModel::GetTicks() {
  if (!initialized_motor) return ticks_discrete;
  UpdateTicks();

  return ticks_discrete;
}

double hw::MotorModel::GetWheelSpeedRadps() {
  double rev_ps = rpm / 60.0;
  return rev_ps * 2 * M_PI;
}

void hw::MotorModel::UpdateTicks() {
  rpm_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dt = rpm_end_time - rpm_start_time;
  rpm_start_time = std::chrono::high_resolution_clock::now();

  double rev_ps = rpm / 60.0;
  double num_revs = rev_ps * dt.count();
  double d_ticks = num_revs * hw::Config::ticks_per_rev;
  ticks_continuous += d_ticks;
  ticks_discrete = std::round(ticks_continuous);
}

void hw::MotorModel::SetWheelSpeedRpm(double rpm) {
  if (!initialized_motor) {
    initialized_motor = true;
    this->rpm = rpm;
    rpm_start_time = std::chrono::high_resolution_clock::now();
    return;
  }
  UpdateTicks();
  this->rpm = rpm;
}

double hw::MotorModel::GetRpm() {
  return rpm;
}

void hw::MotorModel::Clear() {
  ticks_discrete = 0;
  ticks_continuous = 0;
  rpm = 0;
  initialized_motor = false;
}