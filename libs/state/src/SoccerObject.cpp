#include <iostream>
#include "Coordinates.h"
#include "SoccerObject.h"
#include "SystemConfig.h"
#include "Kinematics.h"

void state::InitSoccerObjects(std::vector<state::SoccerObject>& soccer_objects) {
  // Robots
  for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
    std::string name = "robot" + std::to_string(i);
    Eigen::Vector3d robot_position_m(0, i * 1, 0);
    soccer_objects.push_back(
        state::SoccerObject(name, robot_position_m, cfg::SystemConfig::robot_size_m,
                            cfg::SystemConfig::init_robot_velocity_mps,
                            cfg::SystemConfig::init_robot_acceleration_mpsps, 10));
  }

  // Create Ball instance (polymorphic, inherits from SoccerObject)
  state::Ball ball(Eigen::Vector3d(-cfg::SystemConfig::ball_radius_m + 1, cfg::SystemConfig::ball_radius_m, 0));
  ball.name = "ball";
  ball.velocity = cfg::SystemConfig::init_ball_velocity_mps;
  ball.acceleration = cfg::SystemConfig::init_ball_acceleration_mpsps;
  soccer_objects.push_back(ball);
}

bool state::SoccerObject::IsPointInFrontSector(Eigen::Vector2d point) {
  Eigen::Vector3d center = GetCenterPosition();
  Eigen::Vector2d robot_center(center.x(), center.y());

  // Calculate front direction using same coordinate system as ball attachment
  float rotation_rad = position[2]; // Use the z-component as the angle
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
                                  Eigen::Vector2d size_, Eigen::Vector3d velocity_,
                                  Eigen::Vector3d acceleration_, float mass_kg_)
    : name(name_),
      position(position_),
      size(size_),
      velocity(velocity_),
      acceleration(acceleration_),
      mass_kg(mass_kg_) {}

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

  if (robot_manager.GetRobotAction() == rob::RobotAction::DRIBBLE_BALL) {
    // Dribble action is handled by the robot manager's ExecuteDribbleAction method
    // This is called from the main game loop, not here
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

<<<<<<< HEAD
Eigen::Vector3d state::SoccerObject::GetCenterPosition() { return position; }

state::SoccerObject::~SoccerObject() {
    attached_to = nullptr;  
=======
Eigen::Vector3d state::SoccerObject::GetCenterPosition() {
  return Eigen::Vector3d(position[0] + size[0] / 2, position[1] + size[1] / 2, 0); // Z-coordinate should be 0 for 2D
}

// Ball class implementation with proper SSL parameters and physics
state::Ball::Ball() 
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

state::Ball::Ball(const Eigen::Vector3d& position_) : Ball() {
  position = position_;
}

void state::Ball::UpdatePhysics(Eigen::Vector3d& position, Eigen::Vector3d& velocity, 
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

void state::Ball::ApplyKick(Eigen::Vector3d& velocity, const Eigen::Vector2d& kick_direction, 
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

void state::Ball::ApplyDribbleForce(Eigen::Vector3d& velocity, const Eigen::Vector2d& dribble_force) {
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

void state::Ball::ApplySpin(const Eigen::Vector3d& spin_vector) {
    spin_ = spin_vector;
}

Eigen::Vector2d state::Ball::CalculateMagnusForce(const Eigen::Vector3d& velocity, 
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

void state::Ball::HandleBounce(Eigen::Vector3d& velocity, const Eigen::Vector2d& surface_normal, 
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

void state::Ball::ApplyFriction(Eigen::Vector3d& velocity, double friction_coefficient, double dt) {
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

void state::Ball::ApplyAirResistance(Eigen::Vector3d& velocity, double dt) {
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
>>>>>>> b35dac02 (Kick, Dribble, Testing, Demos)
}