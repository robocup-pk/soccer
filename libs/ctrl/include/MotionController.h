#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Eigen/Dense>

#include "SystemConfig.h"

namespace ctrl {

class MotionController {
 public:
  MotionController();
  std::pair<bool, Eigen::Vector3d> DriveToPoint(Eigen::Vector3d pose_fWorld,
                                                Eigen::Vector3d pose_destination);

  std::pair<bool, Eigen::Vector3d> GoToHome(Eigen::Vector3d pose_fWorld,
                                            Eigen::Vector3d pose_destination);

  std::pair<bool, Eigen::Vector3d> InterpolateToPoint(Eigen::Vector3d pose_fWorld,
                                                Eigen::Vector3d pose_destination);

 private:
  double position_error_tolerance_m;
};

}  // namespace ctrl

#endif  // MOTION_CONTROLLER_H