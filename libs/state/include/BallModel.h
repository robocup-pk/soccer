#ifndef BALL_MODEL_H
#define BALL_MODEL_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace state {

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
  
  // Trajectory prediction
  Eigen::Vector2d PredictPosition(const Eigen::Vector3d& current_pos, 
                                  const Eigen::Vector3d& current_vel, 
                                  double prediction_time) const;

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

}  // namespace state

#endif  // BALL_MODEL_H