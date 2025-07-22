#include "BallModel.h"
#include <cmath>
#include <algorithm>

namespace kin {

BallModel::BallModel(double radius, double mass) 
    : radius_(radius), 
      mass_(mass),
      spin_(Eigen::Vector3d::Zero()),
      friction_coefficient_(0.4),    // Realistic grass friction
      air_resistance_(0.47),         // Sphere drag coefficient
      restitution_(0.8),             // Ball bounce factor
      magnus_coefficient_(0.00005)   // Magnus effect strength
{
}

void BallModel::UpdatePhysics(Eigen::Vector3d& position, Eigen::Vector3d& velocity, 
                              Eigen::Vector3d& acceleration, double dt) {
    // Apply air resistance
    ApplyAirResistance(velocity, dt);
    
    // Apply ground friction (only when ball is on ground)
    if (std::abs(position[2]) < radius_) {  // Ball is touching ground
        ApplyFriction(velocity, friction_coefficient_, dt);
    }
    
    // Apply Magnus force (spin effects)
    if (spin_.norm() > 0.1) {  // Only apply if significant spin
        Eigen::Vector2d magnus_force = CalculateMagnusForce(velocity, spin_);
        acceleration[0] += magnus_force[0] / mass_;
        acceleration[1] += magnus_force[1] / mass_;
    }
    
    // Apply gravity (if implementing 3D physics)
    acceleration[2] -= 9.81;  // Gravity in Z direction
    
    // Integrate physics using Euler method
    velocity += acceleration * dt;
    position += velocity * dt;
    
    // Decay spin over time due to air resistance
    spin_ *= (1.0 - 0.01 * dt);  // Spin decay factor
    
    // Reset acceleration for next frame
    acceleration = Eigen::Vector3d::Zero();
}

void BallModel::ApplyKick(Eigen::Vector3d& velocity, const Eigen::Vector2d& kick_direction, 
                          double kick_power) {
    // Normalize kick direction
    Eigen::Vector2d normalized_direction = kick_direction.normalized();
    
    // Apply kick velocity (kick_power is in m/s)
    velocity[0] = normalized_direction[0] * kick_power;
    velocity[1] = normalized_direction[1] * kick_power;
    
    // Add some random spin based on kick direction and power
    double spin_magnitude = kick_power * 0.5;  // Spin proportional to kick power
    spin_[2] = spin_magnitude * (0.5 - static_cast<double>(rand()) / RAND_MAX); // Random spin
    
    // For chip kicks, add vertical component (uncomment for 3D)
    if (kick_power > 3.0) {
        velocity[2] = kick_power * 0.3;  // Vertical component for chip kicks
    }
}

void BallModel::ApplyDribbleForce(Eigen::Vector3d& velocity, const Eigen::Vector2d& dribble_force) {
    // Apply gentle force for dribbling (much weaker than kicks)
    velocity[0] += dribble_force[0] * 0.1;  // Scaled down for control
    velocity[1] += dribble_force[1] * 0.1;
    
    // Limit dribble velocity to realistic range
    double max_dribble_speed = 2.0;  // m/s
    double current_speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    if (current_speed > max_dribble_speed) {
        velocity[0] = (velocity[0] / current_speed) * max_dribble_speed;
        velocity[1] = (velocity[1] / current_speed) * max_dribble_speed;
    }
}

void BallModel::ApplySpin(const Eigen::Vector3d& spin_vector) {
    spin_ = spin_vector;
}

Eigen::Vector2d BallModel::CalculateMagnusForce(const Eigen::Vector3d& velocity, 
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

void BallModel::HandleBounce(Eigen::Vector3d& velocity, const Eigen::Vector2d& surface_normal, 
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

void BallModel::ApplyFriction(Eigen::Vector3d& velocity, double friction_coefficient, double dt) {
    // Ground friction opposes motion
    double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    
    if (speed > 0.01) {  // Avoid division by zero
        // Friction force proportional to normal force (mg) and friction coefficient
        double friction_deceleration = friction_coefficient * 9.81;  // Assuming g = 9.81 m/s²
        double friction_magnitude = friction_deceleration * dt;
        
        // Don't over-apply friction (can't reverse direction)
        friction_magnitude = std::min(friction_magnitude, speed);
        
        // Apply friction opposite to velocity direction
        velocity[0] -= (velocity[0] / speed) * friction_magnitude;
        velocity[1] -= (velocity[1] / speed) * friction_magnitude;
        
        // Reduce spin due to rolling friction
        spin_ *= (1.0 - friction_coefficient * dt * 0.5);
    } else {
        // Stop ball if speed is very low
        velocity[0] = 0.0;
        velocity[1] = 0.0;
        spin_ = Eigen::Vector3d::Zero();
    }
}

void BallModel::ApplyAirResistance(Eigen::Vector3d& velocity, double dt) {
    // Air resistance force = 0.5 * ρ * Cd * A * v²
    // Where ρ = air density, Cd = drag coefficient, A = cross-sectional area, v = velocity
    
    double speed = std::sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1]);
    
    if (speed > 0.1) {  // Only apply at reasonable speeds
        double air_density = CalculateAirDensity();
        double drag_coefficient = CalculateDragCoefficient();
        double cross_sectional_area = M_PI * radius_ * radius_;
        
        // Drag force magnitude
        double drag_force = 0.5 * air_density * drag_coefficient * cross_sectional_area * speed * speed;
        double drag_acceleration = drag_force / mass_;
        double drag_deceleration = drag_acceleration * dt;
        
        // Don't over-apply drag
        drag_deceleration = std::min(drag_deceleration, speed);
        
        // Apply drag opposite to velocity direction
        velocity[0] -= (velocity[0] / speed) * drag_deceleration;
        velocity[1] -= (velocity[1] / speed) * drag_deceleration;
    }
}

Eigen::Vector2d BallModel::PredictPosition(const Eigen::Vector3d& current_pos, 
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

double BallModel::CalculateAirDensity() const {
    // Standard air density at sea level (kg/m³)
    return 1.225;
}

double BallModel::CalculateDragCoefficient() const {
    // Drag coefficient for a smooth sphere
    return air_resistance_;
}

}  // namespace kin