#include <iostream>

#include "Utils.h"
#include "MotionController.h"
#include "SystemConfig.h"

ctrl::MotionController::MotionController() {
  position_error_tolerance_m = 0.01;           // 5 cm
  angle_error_tolerance_rad = 1 * M_PI / 180;  // 5 degrees
}

std::pair<bool, Eigen::Vector3d> ctrl::MotionController::DriveToPoint(
    Eigen::Vector3d pose_fWorld, Eigen::Vector3d pose_dest_fWorld) {
  double Kv = 2;
  double Kw = 0.1;

  // Step 1: Compute position error in world frame
  Eigen::Vector2d position_error_fWorld = pose_dest_fWorld.head<2>() - pose_fWorld.head<2>();
  double angle_error_rad = util::WrapAngle(pose_dest_fWorld[2] - pose_fWorld[2]);

  // Step 2: Check if within position tolerance
  double distance_to_goal = position_error_fWorld.norm();
  if (distance_to_goal < position_error_tolerance_m &&
      std::abs(angle_error_rad) < angle_error_tolerance_rad) {
    std::cout << "[ctrl::MotionController::InterpolateToPoint] Finished I2P" << std::endl;
    return std::make_pair(true, Eigen::Vector3d::Zero());
  }

  // Step 3: Rotate into robot frame
  Eigen::Rotation2D<double> fWorldToBody(-pose_fWorld[2]);
  Eigen::Vector2d position_error_fBody = fWorldToBody * position_error_fWorld;

  // Step 4: Apply proportional control
  Eigen::Vector3d velocity_fBody;
  velocity_fBody[0] = Kv * position_error_fBody[0];
  velocity_fBody[1] = Kv * position_error_fBody[1];
  velocity_fBody[2] = Kw * util::WrapAngle(pose_dest_fWorld[2] - pose_fWorld[2]);

  // Clamp
  velocity_fBody[0] = std::clamp(velocity_fBody[0], -cfg::SystemConfig::max_velocity_fBody_mps[0],
                                 cfg::SystemConfig::max_velocity_fBody_mps[0]);
  velocity_fBody[1] = std::clamp(velocity_fBody[1], -cfg::SystemConfig::max_velocity_fBody_mps[1],
                                 cfg::SystemConfig::max_velocity_fBody_mps[1]);
  velocity_fBody[2] = std::clamp(velocity_fBody[2], -cfg::SystemConfig::max_velocity_fBody_mps[2],
                                 cfg::SystemConfig::max_velocity_fBody_mps[2]);

  return std::make_pair(false, velocity_fBody);
}

std::pair<bool, Eigen::Vector3d> ctrl::MotionController::InterpolateToPoint(
    Eigen::Vector3d pose_fWorld, Eigen::Vector3d pose_dest_fWorld) {
  // Step 1: Compute position error in world frame
  Eigen::Vector2d position_error_fWorld = pose_dest_fWorld.head<2>() - pose_fWorld.head<2>();
  double angle_error_rad = util::WrapAngle(pose_dest_fWorld[2] - pose_fWorld[2]);

  // Step 2: Check if within position tolerance
  double distance_to_goal = position_error_fWorld.norm();
  if (distance_to_goal < position_error_tolerance_m &&
      std::abs(angle_error_rad) < angle_error_tolerance_rad) {
    std::cout << "[ctrl::MotionController::InterpolateToPoint] Finished I2P" << std::endl;
    return std::make_pair(true, Eigen::Vector3d::Zero());
  }

  std::cout << "Position error fWorld: " << position_error_fWorld.transpose() << std::endl;
  std::cout << "Angle error: " << angle_error_rad << std::endl;

  // Step 3: Compute direction in robot frame
  Eigen::Rotation2D<double> fWorldToBody(-pose_fWorld[2]);
  Eigen::Vector2d direction_fBody = fWorldToBody * position_error_fWorld;
  std::cout << "Direction fBody: " << direction_fBody.transpose() << std::endl;

  if (direction_fBody.norm() > 1e-6)
    direction_fBody.normalize();
  else
    direction_fBody.setZero();

  std::cout << "Direction fBody unit: " << direction_fBody.transpose() << std::endl;

  // Step 4: Apply constant velocity in that direction
  Eigen::Vector3d velocity_fBody;
  velocity_fBody[0] = direction_fBody[0] * cfg::SystemConfig::max_velocity_fBody_mps[0];
  velocity_fBody[1] = direction_fBody[1] * cfg::SystemConfig::max_velocity_fBody_mps[1];

  // Step 5: Compute angular velocity with adaptive gain
  double linear_speed = velocity_fBody.head<2>().norm();
  double Kw = (linear_speed < 0.01) ? 10.0 : 1;

  if (distance_to_goal < position_error_tolerance_m) {
    velocity_fBody.head<2>().setZero();  // stop moving forward/sideways
  }

  velocity_fBody[2] =
      std::clamp(Kw * angle_error_rad, -cfg::SystemConfig::max_velocity_fBody_mps[2],
                 cfg::SystemConfig::max_velocity_fBody_mps[2]);

  return std::make_pair(false, velocity_fBody);
}
