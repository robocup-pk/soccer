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
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
             .count() /
         1000.0;
}

Eigen::Vector3d util::RotateAboutZ(Eigen::Vector3d pose, double angle_rad) {
  Eigen::Matrix3d rotation_matrix;
  double c = std::cos(angle_rad);
  double s = std::sin(angle_rad);
  rotation_matrix << c, -s, 0, s, c, 0, 0, 0, 1;
  return rotation_matrix * pose;
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