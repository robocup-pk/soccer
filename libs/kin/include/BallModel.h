#ifndef BALL_MODEL_H
#define BALL_MODEL_H

#include <Eigen/Dense>

namespace kin {

class BallModel {
 public:
  // Constructor
  BallModel(double radius = 0.0215, double mass = 0.43);
  
  // Physics update
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
                    double restitution = 0.8);
  void ApplyFriction(Eigen::Vector3d& velocity, double friction_coefficient, double dt);
  void ApplyAirResistance(Eigen::Vector3d& velocity, double dt);
  
  // Trajectory prediction
  Eigen::Vector2d PredictPosition(const Eigen::Vector3d& current_pos, 
                                  const Eigen::Vector3d& current_vel, 
                                  double prediction_time) const;
  
  // Ball properties
  double GetRadius() const { return radius_; }
  double GetMass() const { return mass_; }
  
  // Physics constants
  void SetFriction(double friction) { friction_coefficient_ = friction; }
  void SetAirResistance(double air_resistance) { air_resistance_ = air_resistance; }
  void SetRestitution(double restitution) { restitution_ = restitution; }

 private:
  // Physical properties
  double radius_;                    // Ball radius (m)
  double mass_;                      // Ball mass (kg)
  Eigen::Vector3d spin_;             // Current spin vector (rad/s)
  
  // Physics constants
  double friction_coefficient_;      // Ground friction coefficient
  double air_resistance_;            // Air resistance coefficient  
  double restitution_;               // Bounce damping factor
  double magnus_coefficient_;        // Magnus effect strength
  
  // Internal calculations
  double CalculateAirDensity() const;
  double CalculateDragCoefficient() const;
};

}  // namespace kin

#endif  // BALL_MODEL_H