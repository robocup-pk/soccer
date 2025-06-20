#ifndef SOCCER_OBJECT_H
#define SOCCER_OBJECT_H

#include <Eigen/Dense>
#include <string>

namespace state {

class SoccerObject {
 public:
  SoccerObject() = default;

  SoccerObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_,
               Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero(),
               Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero(), float mass_kg_ = 1)
      : name(name_),
        position(position_),
        size(size_),
        velocity(velocity_),
        acceleration(acceleration_),
        mass_kg(mass_kg_) {}

  virtual void Move(float dt) {
    velocity += acceleration * dt;
    // TODO: think of a better way
    float friction = 0.1f;
    velocity -= velocity * friction * dt;
    position += velocity * dt;
  }

  inline std::string GetName() const { return name; }
  inline Eigen::Vector3d GetPosition() const { return position; }
  inline Eigen::Vector3d GetVelocity() const { return velocity; }
  inline Eigen::Vector3d GetAcceleration() const { return acceleration; }
  inline Eigen::Vector2d GetSize() const { return size; }
  inline float GetRadius() const { return radius_ft; }
  inline float GetMass() const { return mass_kg; }
  inline Eigen::Vector3d GetCenterPosition() {
    return Eigen::Vector3d(position[0] + size[0] / 2, position[1] - size[1] / 2, position[2]);
  }

  Eigen::Vector3d acceleration;  // vx, vy, angular_acc
  Eigen::Vector3d velocity;      // vx, vy, angular_vel
  Eigen::Vector3d position;      // x, y, angle_rad

  Eigen::Vector2d size;
  float radius_ft;
  float mass_kg;

  std::string name;
};

void InitSoccerObjects(std::vector<SoccerObject>& soccer_objects);

}  // namespace state

#endif  // SOCCER_OBJECT_H