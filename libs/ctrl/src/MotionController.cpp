#include "Utils.h"
#include "MotionController.h"

ctrl::MotionController::MotionController() {
  position_error_tolerance_m = 0.05;  // 5 cm
}

Eigen::Vector3d ctrl::MotionController::DriveToPoint(Eigen::Vector3d pose_fWorld,
                                                     Eigen::Vector3d pose_dest_fWorld) {
  double Kv = 1;
  double Kw = 1;

  // Position error in world frame
  Eigen::Vector2d pose_error_fWorld(pose_dest_fWorld[0] - pose_fWorld[0], pose_dest_fWorld[0] - pose_fWorld[1]);
  if (pose_error_fWorld.norm() < position_error_tolerance_m) {
    return Eigen::Vector3d(0, 0, 0);
  }

  // Position error in body frame
  Eigen::Rotation2D<double> fWorldToBody(-pose_fWorld[2]);
  Eigen::Vector3d pose_error_fBody = fWorldToBody * pose_error_fWorld;

  // Required velocity in body frame
  Eigen::Vector3d velocity_fBody;
  velocity_fBody[0] = Kv * pose_error_fBody[0];
  velocity_fBody[1] = Kv * pose_error_fBody[1];
  velocity_fBody[2] = Kw * util::WrapAngle(pose_error_fBody[2]);

  return velocity_fBody;
}