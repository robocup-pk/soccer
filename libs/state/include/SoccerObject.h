#ifndef SOCCER_OBJECT_H
#define SOCCER_OBJECT_H

#include <Eigen/Dense>
#include <string>

namespace state {

class SoccerObject {
 public:
  SoccerObject() = default;

  virtual void Move(float dt) {}

 private:
  Eigen::Vector3d acceleration;  // vx, vy, angular_acc
  Eigen::Vector3d velocity;      // vx, vy, angular_vel
  Eigen::Vector3d position;      // x, y, angle_rad

  Eigen::Vector2d size;
  float radius_ft;
  float mass_kg;

  std::string name;
};

}  // namespace state

#endif  // SOCCER_OBJECT_H