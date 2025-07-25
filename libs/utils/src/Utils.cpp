#include <chrono>
#include <thread>
#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Dense>
#include "Coordinates.h"
#include "Utils.h"

#ifndef M_PI  // in case it doesnt work on windows
#define M_PI 3.14159265
#endif

float util::PixelsPerMm() {
  // Calculate pixels per mm based on window size and field dimensions
  return cfg::Coordinates::px_per_m * 0.001;
}

float util::MmToPixels(float mm_value) { return mm_value * PixelsPerMm(); }

std::string util::ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Failed to open file: " + path);

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string util::GetExecutableDir() { return CMAKE_BUILD_DIR; }

double util::GetCurrentTime() {
  static const auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - start).count();
}

Eigen::Vector3d util::RotateAboutZ(Eigen::Vector3d pose, double angle_rad) {
  // Eigen::Matrix3d rotation_matrix;
  // double c = std::cos(angle_rad);
  // double s = std::sin(angle_rad);
  // rotation_matrix << c, -s, 0, s, c, 0, 0, 0, 1;
  // return rotation_matrix * pose;
  Eigen::AngleAxisd rotation(angle_rad, Eigen::Vector3d::UnitZ());
  return rotation * pose;
}

double util::WrapAngle(double angle_rad) {
  double rad = std::fmod(angle_rad + M_PI, 2 * M_PI);
  if (rad < 0) rad += 2 * M_PI;
  rad -= M_PI;
  // Normalize -π to +π
  if (std::abs(rad + M_PI) < 1e-8) return M_PI;
  return rad;
}

void util::WaitMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

double util::CalculateDt() {
  static auto start_time = std::chrono::high_resolution_clock::now();
  static auto last_time = start_time;
  auto current_time = std::chrono::high_resolution_clock::now();
  auto duration = current_time - last_time;
  double dt = std::chrono::duration<double>(duration).count();
  last_time = current_time;
  return dt;
}

double util::ComputeAnglefromGyroData(double angular_velocity_radps) {
  static double angle_rad = 0.0;
  static auto prev_time = std::chrono::steady_clock::now();

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - prev_time;
  prev_time = now;

  angle_rad += angular_velocity_radps * dt.count();
  return angle_rad;
}