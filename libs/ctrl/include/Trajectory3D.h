#ifndef TRAJECTORY_3D_H
#define TRAJECTORY_3D_H

#include <Eigen/Dense>

namespace ctrl {

class Trajectory3D {
 public:
  static std::pair<bool, std::optional<Eigen::Vector3d>> IsFeasible(Eigen::Vector3d distance_m,
                                                                    double total_time_s);

  Trajectory3D(Eigen::Vector3d pose_start, Eigen::Vector3d pose_end, double t_start_s,
               double t_end_s);

  Eigen::Vector3d VelocityAtT(double t);

  // Helpers
  void Print();
  Eigen::Vector3d TotalDistance();

  inline double GetTStart() { return t_start_s; }
  inline double GetTFinish() { return t_finish_s; }

 private:
  Eigen::Vector3d pose_start;
  Eigen::Vector3d pose_end;
  double t_start_s;
  double t_finish_s;

  double total_time_s;
  Eigen::Vector3d distance_m_rad;

  // Trajectory limits
  Eigen::Vector3d t_acc_s;
  Eigen::Vector3d v_max;  // computed
};

}  // namespace ctrl

#endif  // TRAJECTORY_3D_H