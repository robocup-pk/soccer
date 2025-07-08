#include "Utils.h"
#include "MotionController.h"

ctrl::MotionController::MotionController() {
  position_error_tolerance_m = 0.05;  // 5 cm
}

std::pair<bool, Eigen::Vector3d> ctrl::MotionController::DriveToPoint(
    Eigen::Vector3d pose_fWorld, Eigen::Vector3d pose_dest_fWorld) {
  double Kv = 2;
  double Kw = 2;

  // Step 1: Compute position error in world frame
  Eigen::Vector2d pose_error_fWorld(pose_dest_fWorld[0] - pose_fWorld[0],
                                    pose_dest_fWorld[1] - pose_fWorld[1]);

  // Step 2: Check if finished motion
  if (pose_error_fWorld.norm() < position_error_tolerance_m) {
    return std::make_pair(true, Eigen::Vector3d(0, 0, 0));
  }

  // Step 3: Rotate into robot frame
  Eigen::Rotation2D<double> fWorldToBody(-pose_fWorld[2]);
  Eigen::Vector2d pose_error_fBody = fWorldToBody * pose_error_fWorld;

  // Step 4: Apply proportional control
  Eigen::Vector3d velocity_fBody;
  velocity_fBody[0] = Kv * pose_error_fBody[0];
  velocity_fBody[1] = Kv * pose_error_fBody[1];
  velocity_fBody[2] = Kw * util::WrapAngle(pose_dest_fWorld[2] - pose_fWorld[2]);

  return std::make_pair(false, velocity_fBody);
}