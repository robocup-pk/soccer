#ifndef TRAJECTORY_3D_H
#define TRAJECTORY_3D_H

#include <optional>

#include <Eigen/Dense>

namespace ctrl {

class Trajectory3D {
 public:
  static std::pair<bool, std::optional<Eigen::Vector3d>> IsFeasible(
      Eigen::Vector3d distance_m, double total_time_s,
      Eigen::Vector3d v0 = Eigen::Vector3d::Zero());

  virtual Eigen::Vector3d VelocityAtT(double t) = 0;

  virtual Eigen::Vector3d PositionAtT(double t) = 0;

  // Helpers
  virtual void Print() = 0;
  virtual Eigen::Vector3d TotalDistance() = 0;

  virtual void SetTFinish(double t_finish_s) = 0;
  virtual double GetTStart() = 0;
  virtual double GetTFinish() = 0;
  virtual Eigen::Vector3d GetPoseStart() = 0;
  virtual Eigen::Vector3d GetPoseEnd() = 0;

  virtual ~Trajectory3D() = default;
};

}  // namespace ctrl

#endif  // TRAJECTORY_3D_H