#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <Eigen/Dense>
#include <memory>

#include "RobotModel.h"

namespace est {
class StateEstimator {
 public:
  StateEstimator();

  void NewMotorsData(Eigen::Vector4d wheel_speeds_rpm);
  void NewGyroData(double w_radps);
  void NewCameraData(Eigen::Vector3d pose_meas);

  void InitializePose(Eigen::Vector3d pose_fWorld_init);

  Eigen::Vector3d GetPose();
  Eigen::Vector3d GetPoseInit();
  void SetPose(Eigen::Vector3d pose);

  ~StateEstimator();

  bool initialized_pose;

 private:
  // Encoders
  bool initialized_encoder;
  double t_last_encoder;

  // Gyro
  bool initialized_gyro;
  double angle_random_walk_per_rt_t;
  double t_last_gyro;

  // Pose
  Eigen::Vector3d pose_init;
  Eigen::Vector3d pose_est;

  // Noise
  // P
  double init_sigma_m;
  double init_sigma_rad;
  Eigen::Matrix3d state_cov;
  // Q
  double process_sigma_m;
  double process_sigma_rad;
  Eigen::Matrix3d process_cov;
  // R
  double meas_sigma_m;
  double meas_sigma_rad;
  Eigen::Matrix3d meas_cov;

  std::shared_ptr<kin::RobotModel> robot_model;
};
}  // namespace est

#endif  // ESTIMATOR_H