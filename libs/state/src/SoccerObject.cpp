#include <iostream>
#include "Coordinates.h"
#include "SoccerObject.h"
#include "SystemConfig.h"
#include "Kinematics.h"
#include "BallModel.h"
#include "RobotManager.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  // Robots (team one)
  for (int i = 0; i < cfg::SystemConfig::num_robots / 2; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_m =
        i < cfg::RightRobotHomeCoordinates.size()
            ? cfg::RightRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(i))
            : Eigen::Vector3d(0.3, 0, 0);
    soccer_objects.push_back(state::SoccerObject(
        name, robot_position_m, cfg::SystemConfig::robot_size_m, 1,
        cfg::SystemConfig::init_robot_velocity_mps,
        cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
  }

  // Robots (team two)
  for (int i = cfg::SystemConfig::num_robots / 2; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    int index = i - cfg::SystemConfig::num_robots / 2;
    Eigen::Vector3d robot_position_m =
        index < cfg::LeftRobotHomeCoordinates.size()
            ? cfg::LeftRobotHomeCoordinates.at(static_cast<cfg::RobotHomePosition>(index))
            : Eigen::Vector3d(-0.3, 0, 0);
    soccer_objects.push_back(state::SoccerObject(
        name, robot_position_m, cfg::SystemConfig::robot_size_m, 2,
        cfg::SystemConfig::init_robot_velocity_mps,
        cfg::SystemConfig::init_robot_acceleration_mpsps, cfg::SystemConfig::robot_mass_kg));
  }

  // Ball
  soccer_objects.push_back(state::SoccerObject(
      "ball", cfg::SystemConfig::init_ball_position,
      Eigen::Vector2d(cfg::SystemConfig::ball_radius_m * 2, cfg::SystemConfig::ball_radius_m * 2),
      0, cfg::SystemConfig::init_ball_velocity_mps,
      cfg::SystemConfig::init_ball_acceleration_mpsps, cfg::SystemConfig::ball_mass_kg));
}

bool state::SoccerObject::IsPointInFrontSector(Eigen::Vector2d point) {
  Eigen::Vector3d center = GetCenterPosition();
  Eigen::Vector2d robot_center(center.x(), center.y());

  // Calculate front direction using same coordinate system as ball attachment
  float rotation_rad = position[2]; // Use the z-component as the angle
  Eigen::Vector2d front_dir(cos(rotation_rad), sin(rotation_rad));

  // Vector from robot center to point
  Eigen::Vector2d to_point = point - robot_center;
  float distance = to_point.norm();

  // Check if point is within maximum detection distance
  const float MAX_DISTANCE = 0.3f; // 30cm
  if (distance > MAX_DISTANCE) return false;

  // Check if point is in the front sector (120 degree cone)
  float dot_product = front_dir.dot(to_point.normalized());
  const float ANGLE_THRESHOLD = cos(60.0f * M_PI / 180.0f); // 60 degrees = half of 120 degree cone

  return dot_product > ANGLE_THRESHOLD;
}

Eigen::Vector3d state::SoccerObject::GetCenterPosition() {
  return position;
}

void state::SoccerObject::SetRobotRole(state::SoccerObject::Role r) {
  role = r;
}

void state::SoccerObject::Move(float dt) {
  // Basic kinematic update
  velocity += acceleration * dt;
  position += velocity * dt;
}

// Name-only constructor
state::SoccerObject::SoccerObject(std::string name_) 
  : name(name_) {
  // Initialize with defaults
  acceleration = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  position = Eigen::Vector3d::Zero();
  size = Eigen::Vector2d::Zero();
  radius_m = 0.0;
  mass_kg = 1.0;
  team_id = 0;
  is_attached = false;
  was_given_speeding_foul_in_stop = false;
  is_selected_player = false;
  attached_to = nullptr;
  role = Role::Unassigned;
  is_dribbling = false;
}

// Full constructor (version with team_id)
state::SoccerObject::SoccerObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_, int team_id_,
                                  Eigen::Vector3d velocity_, Eigen::Vector3d acceleration_, float mass_kg_)
  : acceleration(acceleration_),
    velocity(velocity_),
    position(position_),
    size(size_),
    radius_m(size_.x() / 2.0),
    mass_kg(mass_kg_),
    team_id(team_id_),
    name(name_),
    is_attached(false),
    was_given_speeding_foul_in_stop(false),
    is_selected_player(false),
    attached_to(nullptr),
    role(Role::Unassigned),
    is_dribbling(false) {}

// Constructor without team_id
state::SoccerObject::SoccerObject(std::string name_, Eigen::Vector3d position_, Eigen::Vector2d size_,
                                  Eigen::Vector3d velocity_, Eigen::Vector3d acceleration_, float mass_kg_)
  : acceleration(acceleration_),
    velocity(velocity_),
    position(position_),
    size(size_),
    radius_m(size_.x() / 2.0),
    mass_kg(mass_kg_),
    team_id(0),
    name(name_),
    is_attached(false),
    was_given_speeding_foul_in_stop(false),
    is_selected_player(false),
    attached_to(nullptr),
    role(Role::Unassigned),
    is_dribbling(false) {}

// RobotManager constructor
state::SoccerObject::SoccerObject(const rob::RobotManager& robot_manager)
  : name("robot"),
    team_id(1),
    mass_kg(3.0) {
  // Initialize with robot manager data
  position = robot_manager.GetPoseInWorldFrame();
  velocity = robot_manager.GetBodyVelocity();
  acceleration = Eigen::Vector3d::Zero();
  size = Eigen::Vector2d(0.18, 0.18); // Standard robot size
  radius_m = 0.09;
  is_attached = false;
  was_given_speeding_foul_in_stop = false;
  is_selected_player = false;
  attached_to = nullptr;
  role = Role::Unassigned;
  is_dribbling = false;
}

// Virtual destructor
state::SoccerObject::~SoccerObject() = default;

// Assignment from RobotManager
state::SoccerObject& state::SoccerObject::operator=(rob::RobotManager& robot_manager) {
  position = robot_manager.GetPoseInWorldFrame();
  velocity = robot_manager.GetVelocityInWorldFrame();
  return *this;
}