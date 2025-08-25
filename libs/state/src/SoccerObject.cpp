#include <iostream>
#include "Coordinates.h"
#include "SoccerObject.h"
#include "SystemConfig.h"
#include "Kinematics.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  for (int i = 0; i < cfg::SystemConfig::num_robots / 2; ++i) {
    // Robots (team one)
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_m =
        i < cfg::RightRobotHomeCoordinates.size()
            ? cfg::RightRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(i))
            : Eigen::Vector3d(0.3, 0, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_m, cfg::SystemConfig::robot_size_m, 1,
                            cfg::SystemConfig::init_robot_velocity_mps,
                            cfg::SystemConfig::init_robot_acceleration_mpsps, 10));
  }

  // Robots (team two)
  for (int i = cfg::SystemConfig::num_robots / 2; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    int index = i - cfg::SystemConfig::num_robots / 2;
    Eigen::Vector3d robot_position_m =
        index < cfg::LeftRobotHomeCoordinates.size()
            ? cfg::LeftRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(index))
            : Eigen::Vector3d(-0.3, 0, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_m, cfg::SystemConfig::robot_size_m, 2,
                            cfg::SystemConfig::init_robot_velocity_mps,
                            cfg::SystemConfig::init_robot_acceleration_mpsps, 10));
  }

  // ball
  soccer_objects.push_back(state::SoccerObject(
      "ball", cfg::SystemConfig::init_ball_position,
      Eigen::Vector2d(cfg::SystemConfig::ball_radius_m * 2, cfg::SystemConfig::ball_radius_m * 2),
      0, cfg::SystemConfig::init_ball_velocity_mps,
      cfg::SystemConfig::init_ball_acceleration_mpsps, 1));
}

bool state::SoccerObject::IsPointInFrontSector(Eigen::Vector2d point) {
  Eigen::Vector3d center = GetCenterPosition();
  Eigen::Vector2d robot_center(center.x(), center.y());

  // Calculate front direction using same coordinate system as ball attachment
  float rotation_rad = (position[2]);
  Eigen::Vector2d front_dir(cos(rotation_rad), sin(rotation_rad));

  Eigen::Vector2d to_point = point - robot_center;
  float forward_component = to_point.dot(front_dir);

  if (forward_component <= 0) {
    return false;
  }

  float dot_product = to_point.normalized().dot(front_dir);
  float angle_threshold = cos(M_PI / 4.0f);

  return dot_product > angle_threshold;
}

state::SoccerObject::SoccerObject(std::string name_, Eigen::Vector3d position_,
                                  Eigen::Vector2d size_, int team_id, Eigen::Vector3d velocity_,
                                  Eigen::Vector3d acceleration_, float mass_kg_)
    : name(name_),
      position(position_),
      size(size_),
      team_id(team_id),
      velocity(velocity_),
      acceleration(acceleration_),
      mass_kg(mass_kg_),
      radius_m(cfg::SystemConfig::robot_size_m[0] / 2) {
  if (name == "ball") radius_m = cfg::SystemConfig::ball_radius_m;
}

state::SoccerObject::SoccerObject(std::string name_, Eigen::Vector3d position_,
                                  Eigen::Vector2d size_, Eigen::Vector3d velocity_,
                                  Eigen::Vector3d acceleration_, float mass_kg_)
    : name(name_),
      position(position_),
      size(size_),
      team_id(0),
      velocity(velocity_),
      acceleration(acceleration_),
      mass_kg(mass_kg_),
      radius_m(cfg::SystemConfig::robot_size_m[0] / 2) {
  if (name == "ball") radius_m = cfg::SystemConfig::ball_radius_m;
}

state::SoccerObject::SoccerObject(const rob::RobotManager& robot_manager) {
  position = robot_manager.GetPoseInWorldFrame();  // TODO: Center position
  velocity = robot_manager.GetVelocityInWorldFrame();
}

state::SoccerObject& state::SoccerObject::operator=(rob::RobotManager& robot_manager) {
  // Center position
  position = robot_manager.GetPoseInWorldFrame();
  position[0] += size[0] / 2;
  position[0] -= size[1] / 2;
  velocity = robot_manager.GetVelocityInWorldFrame();
  if (robot_manager.GetRobotAction() == rob::RobotAction::PASS_BALL && this->name != "ball" &&
      this->attached_to) {
    kin::DetachBall(*this->attached_to, 1.0f);
  }

  if (robot_manager.GetRobotAction() == rob::RobotAction::KICK_BALL && this->name != "ball" &&
      this->attached_to) {
    kin::DetachBall(*this->attached_to, 6.5f);
  }

  robot_manager.SetRobotAction(rob::RobotAction::MOVE);
  return *this;
}

void state::SoccerObject::Move(float dt) {
  // TODO: Improve
  if (name == "ball" && is_attached) return;
  velocity += acceleration * dt;
  float friction = 0.9f;
  velocity -= velocity * friction * dt;
  position += velocity * dt;
  position[2] = util::WrapAngle(position[2]);
}

Eigen::Vector3d state::SoccerObject::GetCenterPosition() { return position; }

void state::SoccerObject::SetRobotRole(state::SoccerObject::Role r) { role = r; }

state::SoccerObject::~SoccerObject() { attached_to = nullptr; }