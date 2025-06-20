#include "ReadFile.h"

std::string util::ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Failed to open file: " + path);

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string util::GetExecutableDir() {
  std::string exec_dir = "/home/duneai/Desktop/robocup/ros_ws/build/soccer_ros/soccer_build/";
  return exec_dir;  // std::filesystem::current_path().string();
}