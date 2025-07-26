#ifndef SOCCER_OBJECT_H
#define SOCCER_OBJECT_H

#include <Eigen/Dense>
#include <string>

#include "RobotManager.h"

namespace state {

class SoccerObject {
 public:
  // Constructors
  SoccerObject() = default;
  SoccerObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_,
               Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero(),
               Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero(), float mass_kg_ = 1);
  SoccerObject(const rob::RobotManager& robot_manager);
  SoccerObject& operator=(rob::RobotManager& robot_manager);
  virtual ~SoccerObject();

  // Getters
  inline Eigen::Vector3d GetPosition() const { return position; }
  Eigen::Vector3d GetCenterPosition();

  // Kinematics
  bool IsPointInFrontSector(Eigen::Vector2d point);
  virtual void Move(float dt);

  Eigen::Vector3d acceleration;  // vx, vy, angular_acc
  Eigen::Vector3d velocity;      // vx, vy, angular_vel
  Eigen::Vector3d position;      // x, y, angle_rad

  Eigen::Vector2d size;
  float radius_m;
  float mass_kg;

  std::string name;

  // Ball Attachment
  bool is_attached = false;
  bool is_selected_player = false;
  SoccerObject* attached_to;

  // Dribbling State (Physics-based ball control)
  bool is_dribbling = false;  // When true, robot uses dribble physics instead of holding
};

// Forward declaration - Ball class is defined in BallModel.h
class Ball;

void InitSoccerObjects(std::vector<SoccerObject>& soccer_objects);

}  // namespace state

#endif  // SOCCER_OBJECT_H