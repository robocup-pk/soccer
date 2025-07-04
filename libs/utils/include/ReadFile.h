#ifndef READ_FILE_H
#define READ_FILE_H

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <filesystem>

#include <Eigen/Dense>

namespace util {
  std::string ReadFile(const std::string& path);
  std::string GetExecutableDir();
  Eigen::Vector3d RotateAboutZ(Eigen::Vector3d pose, double angle_rad);
  double WrapAngle(double angle_rad);
  double GetCurrentTime();
  void WaitMs(int ms);
}  // namespace util

#endif  // READ_FILE_H