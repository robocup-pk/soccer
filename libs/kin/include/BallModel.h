#ifndef BALL_MODEL_H
#define BALL_MODEL_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace kin {

class BallModel : public state::SoccerObject {
 public:
  // Constructor - inherits mass and radius from SoccerObject
  BallModel();
  BallModel(const Eigen::Vector3d& position_);
  
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
  
  // Ball properties - inherited from SoccerObject
  double GetRadius() const { return radius_m; }
  double GetMass() const { return mass_kg; }
  
  // Physics constants
  void SetFriction(double friction) { friction_coefficient_ = friction; }
  void SetAirResistance(double air_resistance) { air_resistance_ = air_resistance; }
  void SetRestitution(double restitution) { restitution_ = restitution; }

 private:
  // Physical properties (radius_m and mass_kg inherited from SoccerObject)
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