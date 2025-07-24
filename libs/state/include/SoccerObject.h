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
  ~SoccerObject();

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

// Ball class inheriting from SoccerObject with proper mass and radius and physics
class Ball : public SoccerObject {
 public:
  Ball();
  Ball(const Eigen::Vector3d& position_);
  
  // Physics update method
  void UpdatePhysics(Eigen::Vector3d& position, Eigen::Vector3d& velocity, 
                     Eigen::Vector3d& acceleration, double dt);
  
  // Ball interaction methods
  void ApplyKick(Eigen::Vector3d& velocity, const Eigen::Vector2d& kick_direction, 
                 double kick_power);
  void ApplyDribbleForce(Eigen::Vector3d& velocity, const Eigen::Vector2d& dribble_force);
  
  // Spin effects
  void ApplySpin(const Eigen::Vector3d& spin_vector);
  Eigen::Vector2d CalculateMagnusForce(const Eigen::Vector3d& velocity, 
                                       const Eigen::Vector3d& spin) const;
  
  // Surface interactions
  void HandleBounce(Eigen::Vector3d& velocity, const Eigen::Vector2d& surface_normal, 
                    double restitution = 0.6);
  
  // Physics parameters
  Eigen::Vector3d GetSpin() const { return spin_; }
  double GetFriction() const { return friction_coefficient_; }
  double GetRestitution() const { return restitution_; }

private:
  // Physics state
  Eigen::Vector3d spin_;              // Angular velocity (rad/s)
  
  // Material properties (SSL-specific)
  double friction_coefficient_;       // Rolling friction
  double air_resistance_;            // Air drag coefficient  
  double restitution_;               // Bounce coefficient
  double magnus_coefficient_;        // Spin effect strength
  
  // Physics helper methods
  void ApplyFriction(Eigen::Vector3d& velocity, double friction_coeff, double dt);
  void ApplyAirResistance(Eigen::Vector3d& velocity, double dt);
};

void InitSoccerObjects(std::vector<SoccerObject>& soccer_objects);

}  // namespace state

#endif  // SOCCER_OBJECT_H