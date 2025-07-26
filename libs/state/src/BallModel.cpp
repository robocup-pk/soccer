#include "BallModel.h"
#include "SystemConfig.h"
#include <cmath>
#include <algorithm>

namespace state {

// Ball class implementation with proper SSL parameters and physics
Ball::Ball() 
    : SoccerObject("ball", Eigen::Vector3d::Zero(), 
                   Eigen::Vector2d(cfg::SystemConfig::ball_radius_m * 2, 
                                  cfg::SystemConfig::ball_radius_m * 2),
                   Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.046f),
      spin_(Eigen::Vector3d::Zero()),
      friction_coefficient_(3.5),    // High friction for SSL carpet/turf - ball stops quickly
      air_resistance_(0.1),          // Minimal air resistance for SSL (small field)
      restitution_(0.6),             // Lower restitution for SSL ball
      magnus_coefficient_(0.0001)    // Minimal spin effect for SSL
{
  radius_m = cfg::SystemConfig::ball_radius_m;  // SSL standard: 21.5mm radius
  mass_kg = 0.046f;  // SSL standard: 46g mass
}

Ball::Ball(const Eigen::Vector3d& position_) : Ball() {
  position = position_;
}

void Ball::UpdatePhysics(Eigen::Vector3d& position, Eigen::Vector3d& velocity, 
                                Eigen::Vector3d& acceleration, double dt) {
    // SSL Physics: Ball always stays on ground (2D simulation)
    position[2] = 0.0;  // Force ball to stay on ground
    velocity[2] = 0.0;  // No vertical velocity
    acceleration[2] = 0.0;  // No vertical acceleration (no gravity in 2D)
    
    // Apply ground friction (always active in SSL)
    ApplyFriction(velocity, friction_coefficient_, dt);
    
    // Apply minimal air resistance (SSL field is small)
    ApplyAirResistance(velocity, dt);
    
    // Apply Magnus force (minimal for SSL)
    if (spin_.norm() > 0.1) {  // Only apply if significant spin
        Eigen::Vector2d magnus_force = CalculateMagnusForce(velocity, spin_);
        acceleration[0] += magnus_force[0] / mass_kg;
        acceleration[1] += magnus_force[1] / mass_kg;
    }
    
    // Integrate physics using Euler method (2D only)
    velocity[0] += acceleration[0] * dt;
    velocity[1] += acceleration[1] * dt;
    position[0] += velocity[0] * dt;
    position[1] += velocity[1] * dt;
    
    // Decay spin over time
    spin_ *= (1.0 - 0.05 * dt);  // Faster spin decay for SSL
    
    // Reset acceleration for next frame
    acceleration = Eigen::Vector3d::Zero();
}

void Ball::ApplyKick(Eigen::Vector3d& velocity, const Eigen::Vector2d& kick_direction, 
                           double kick_power) {
    // Normalize kick direction
    Eigen::Vector2d normalized_direction = kick_direction.normalized();
    
    // Apply kick velocity (kick_power is in m/s) - SSL realistic speeds
    // Typical SSL robot kick: 2-6 m/s max
    velocity[0] = normalized_direction[0] * kick_power;
    velocity[1] = normalized_direction[1] * kick_power;
    velocity[2] = 0.0;  // Always 2D for SSL
    
    // Add minimal spin for SSL (robots have precise control)
    double spin_magnitude = kick_power * 0.1;  // Reduced spin for SSL
    spin_[2] = spin_magnitude * (0.5 - static_cast<double>(rand()) / RAND_MAX); // Random spin
    
    // No chip kicks in SSL simulation (always 2D)
}

void Ball::ApplyDribbleForce(Eigen::Vector3d& velocity, const Eigen::Vector2d& dribble_force) {
    // Apply stronger force for dribbling to keep up with robot movement
    velocity[0] += dribble_force[0] * 0.2;  // Increased scaling for better control
    velocity[1] += dribble_force[1] * 0.2;
    
    // Limit dribble velocity to realistic range (increased for SSL)
    double max_dribble_speed = 3.0;  // m/s - higher limit for SSL dribbling
    double current_speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    if (current_speed > max_dribble_speed) {
        velocity[0] = (velocity[0] / current_speed) * max_dribble_speed;
        velocity[1] = (velocity[1] / current_speed) * max_dribble_speed;
    }
}

void Ball::ApplySpin(const Eigen::Vector3d& spin_vector) {
    spin_ = spin_vector;
}

Eigen::Vector2d Ball::CalculateMagnusForce(const Eigen::Vector3d& velocity, 
                                                 const Eigen::Vector3d& spin) const {
    // Magnus force = k * (spin × velocity)
    // In 2D, we only consider Z-axis spin affecting X-Y motion
    double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    
    if (speed < 0.1) return Eigen::Vector2d::Zero();  // No effect at low speeds
    
    // Magnus force perpendicular to velocity direction
    Eigen::Vector2d magnus_force;
    magnus_force[0] = -velocity[1] * spin[2] * magnus_coefficient_ * speed;
    magnus_force[1] = velocity[0] * spin[2] * magnus_coefficient_ * speed;
    
    return magnus_force;
}

void Ball::HandleBounce(Eigen::Vector3d& velocity, const Eigen::Vector2d& surface_normal, 
                              double restitution) {
    // Reflect velocity off surface with restitution
    Eigen::Vector2d velocity_2d(velocity[0], velocity[1]);
    Eigen::Vector2d normal = surface_normal.normalized();
    
    // Calculate reflection
    Eigen::Vector2d reflected = velocity_2d - 2.0 * velocity_2d.dot(normal) * normal;
    
    // Apply restitution (energy loss)
    velocity[0] = reflected[0] * restitution;
    velocity[1] = reflected[1] * restitution;
    
    // Add some random spin from bounce
    double bounce_spin = velocity_2d.norm() * 0.1;
    spin_[2] += bounce_spin * (0.5 - static_cast<double>(rand()) / RAND_MAX);
}

void Ball::ApplyFriction(Eigen::Vector3d& velocity, double friction_coefficient, double dt) {
    // SSL Ground friction - high deceleration on carpet/turf
    double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    
    if (speed > 0.05) {  // Higher threshold for SSL (0.05 m/s = 5 cm/s)
        // SSL friction is much higher than grass - ball decelerates quickly
        double friction_deceleration = friction_coefficient;  // Direct deceleration rate (m/s²)
        double friction_magnitude = friction_deceleration * dt;
        
        // Don't over-apply friction (can't reverse direction)
        friction_magnitude = std::min(friction_magnitude, speed);
        
        // Apply friction opposite to velocity direction
        velocity[0] -= (velocity[0] / speed) * friction_magnitude;
        velocity[1] -= (velocity[1] / speed) * friction_magnitude;
        
        // Reduce spin due to rolling friction
        spin_ *= (1.0 - 0.2 * dt);  // Faster spin decay for SSL carpet
    } else {
        // Stop ball completely when speed is very low (SSL precision)
        velocity[0] = 0.0;
        velocity[1] = 0.0;
        velocity[2] = 0.0;
        spin_ = Eigen::Vector3d::Zero();
    }
}

void Ball::ApplyAirResistance(Eigen::Vector3d& velocity, double dt) {
    // Minimal air resistance for SSL (small field, low speeds)
    double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    
    if (speed > 0.5) {  // Only apply at higher speeds (>0.5 m/s)
        // Simple air resistance proportional to velocity
        double air_resistance_deceleration = air_resistance_ * speed;
        double drag_magnitude = air_resistance_deceleration * dt;
        
        // Don't over-apply drag
        drag_magnitude = std::min(drag_magnitude, speed);
        
        // Apply drag opposite to velocity direction
        velocity[0] -= (velocity[0] / speed) * drag_magnitude;
        velocity[1] -= (velocity[1] / speed) * drag_magnitude;
    }
}

Eigen::Vector2d Ball::PredictPosition(const Eigen::Vector3d& current_pos, 
                                      const Eigen::Vector3d& current_vel, 
                                      double prediction_time) const {
    // Simple prediction assuming constant deceleration due to friction
    double friction_deceleration = friction_coefficient_ * 9.81;
    double current_speed = std::sqrt(current_vel[0]*current_vel[0] + current_vel[1]*current_vel[1]);
    
    if (current_speed < 0.01) {
        return Eigen::Vector2d(current_pos[0], current_pos[1]);  // Ball is stationary
    }
    
    // Time to stop due to friction
    double time_to_stop = current_speed / friction_deceleration;
    double actual_time = std::min(prediction_time, time_to_stop);
    
    // Calculate position using kinematic equation
    Eigen::Vector2d direction(current_vel[0] / current_speed, current_vel[1] / current_speed);
    double distance = current_speed * actual_time - 0.5 * friction_deceleration * actual_time * actual_time;
    
    return Eigen::Vector2d(current_pos[0] + direction[0] * distance, 
                          current_pos[1] + direction[1] * distance);
}

}  // namespace state