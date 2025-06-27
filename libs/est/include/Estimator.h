#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <memory>

#include "RobotModel.h"

namespace est {
class Estimator {
 public:
  Estimator();
  void NewEncoderData(Eigen::Vector4d ticks);
  void NewGyroData(double w_radps);
  void NewCameraData(Eigen::Vector3d pose_meas);

  Eigen::Vector3d GetPose();

  bool initialized_pose;
  kin::RobotDescription robot_desc;
  std::unique_ptr<kin::RobotModel> robot_model;
 private:
  bool initialized_encoder;
  double t_last_encoder;
  double t_last_gyro;

  Eigen::Vector4d last_ticks;

  Eigen::Vector3d pose_est;
  Eigen::Matrix3d state_cov;    // P
  Eigen::Matrix3d process_cov;  // Q
  Eigen::Matrix3d meas_cov;     // R

  double init_sigma_m;
  double init_sigma_rad;
  double process_sigma_m;
  double process_sigma_rad;

  double meas_sigma_m;
  double meas_sigma_rad;

  bool initialized_gyro;

  double angle_random_walk_per_rt_t;
};
}  // namespace est

#endif  // ESTIMATOR_H