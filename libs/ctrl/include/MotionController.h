#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Eigen/Dense>

namespace ctrl {

class MotionController {
 public:
  MotionController();
  Eigen::Vector3d DriveToPoint(Eigen::Vector3d pose_fWorld, Eigen::Vector3d pose_destination);

 private:
  double position_error_tolerance_m;
};

}  // namespace ctrl

#endif  // MOTION_CONTROLLER_H