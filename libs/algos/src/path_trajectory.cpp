#include "path_trajectory.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <cstdlib>

namespace algos {

PathTrajectory::PathTrajectory() 
    : time_step_(0.1), 
      lookahead_distance_(50.0),
      obstacle_weight_(1000.0),
      goal_weight_(100.0),
      rrt_step_size_(15.0),
      goal_bias_(0.1),
      rewire_radius_(30.0) {
}

PathTrajectory::~PathTrajectory() = default;

PathTrajectory::PathResult PathTrajectory::findPath(const Vector2D& start_pos, double start_angle,
                                                   const Vector2D& target_pos, double target_angle,
                                                   const std::vector<Obstacle>& obstacles,
                                                   double max_speed, double max_acceleration) {
    
    // Try RRT first (best for obstacle avoidance)
    PathResult result = findRRTPath(start_pos, target_pos, obstacles, max_speed);
    
    if (!result.success) {
        // Fallback to direct path if RRT fails
        result = findDirectPath(start_pos, target_pos, obstacles, max_speed);
    }
    
    if (!result.success) {
        // Last resort: potential field
        result = findPotentialFieldPath(start_pos, target_pos, obstacles, max_speed);
    }
    
    return result;
}

PathTrajectory::PathResult PathTrajectory::findDirectPath(const Vector2D& start_pos, 
                                                         const Vector2D& target_pos,
                                                         const std::vector<Obstacle>& obstacles,
                                                         double max_speed) {
    PathResult result;
    
    // Check if direct path is collision-free
    if (isCollisionFree(start_pos, target_pos, obstacles)) {
        // Simple direct trajectory
        Vector2D direction = (target_pos - start_pos).normalize();
        double distance = calculateDistance(start_pos, target_pos);
        double travel_time = distance / max_speed;
        
        // Create trajectory points
        for (double t = 0; t <= travel_time; t += time_step_) {
            Vector2D pos = start_pos + direction * (max_speed * t);
            Vector2D vel = direction * max_speed;
            result.trajectory.push_back(TrajectoryPoint(pos, vel, t));
        }
        
        // Add final point
        result.trajectory.push_back(TrajectoryPoint(target_pos, Vector2D(0, 0), travel_time));
        
        result.success = true;
        result.total_time = travel_time;
        result.total_distance = distance;
    } else {
        result.success = false;
    }
    
    return result;
}

PathTrajectory::PathResult PathTrajectory::findPotentialFieldPath(const Vector2D& start_pos,
                                                                 const Vector2D& target_pos,
                                                                 const std::vector<Obstacle>& obstacles,
                                                                 double max_speed) {
    PathResult result;
    
    Vector2D current_pos = start_pos;
    Vector2D current_vel(0, 0);
    double current_time = 0.0;
    double total_distance = 0.0;
    
    const int max_iterations = 1000;
    const double goal_threshold = 5.0;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Check if we've reached the goal
        if (calculateDistance(current_pos, target_pos) < goal_threshold) {
            result.success = true;
            break;
        }
        
        // Calculate forces
        Vector2D total_force = calculateTotalForce(current_pos, target_pos, obstacles);
        
        // Convert force to acceleration (F = ma, assume m = 1)
        Vector2D acceleration = total_force;
        
        // Update velocity with acceleration
        current_vel = current_vel + acceleration * time_step_;
        
        // Limit velocity to max_speed
        current_vel = limitVector(current_vel, max_speed);
        
        // Update position
        Vector2D next_pos = current_pos + current_vel * time_step_;
        
        // Check for collision at next position
        bool collision_detected = false;
        for (const auto& obstacle : obstacles) {
            if (calculateDistance(next_pos, obstacle.position) < obstacle.radius + 10.0) {
                collision_detected = true;
                break;
            }
        }
        
        if (collision_detected) {
            // Apply stronger repulsive force
            Vector2D repulsive_total(0, 0);
            for (const auto& obstacle : obstacles) {
                repulsive_total = repulsive_total + calculateRepulsiveForce(current_pos, obstacle) * 2.0;
            }
            current_vel = repulsive_total.normalize() * max_speed * 0.5;
            next_pos = current_pos + current_vel * time_step_;
        }
        
        // Store trajectory point
        result.trajectory.push_back(TrajectoryPoint(current_pos, current_vel, current_time));
        
        // Update for next iteration
        total_distance += calculateDistance(current_pos, next_pos);
        current_pos = next_pos;
        current_time += time_step_;
    }
    
    // Add final point
    result.trajectory.push_back(TrajectoryPoint(target_pos, Vector2D(0, 0), current_time));
    
    result.total_time = current_time;
    result.total_distance = total_distance;
    
    if (!result.success && result.trajectory.size() > 10) {
        // Consider partial success if we made significant progress
        double progress = calculateDistance(start_pos, current_pos) / calculateDistance(start_pos, target_pos);
        if (progress > 0.8) {
            result.success = true;
        }
    }
    
    return result;
}

PathTrajectory::PathResult PathTrajectory::findRRTPath(const Vector2D& start_pos,
                                                      const Vector2D& target_pos,
                                                      const std::vector<Obstacle>& obstacles,
                                                      double max_speed,
                                                      int max_iterations) {
    PathResult result;
    
    // Define search bounds - make them larger for better exploration
    Vector2D min_bounds(-600, -500);
    Vector2D max_bounds(600, 500);
    
    // Initialize RRT tree
    std::vector<RRTNode> nodes;
    
    // Check if start position is valid (not inside an obstacle)
    for (const auto& obstacle : obstacles) {
        if (calculateDistance(start_pos, obstacle.position) < obstacle.radius + 10.0) {
            std::cout << "[RRT] Warning: Start position too close to obstacle!" << std::endl;
            // Try to find a nearby valid start position
            for (int attempts = 0; attempts < 10; ++attempts) {
                double angle = attempts * M_PI / 5.0;
                Vector2D new_start = start_pos + Vector2D(cos(angle) * 15.0, sin(angle) * 15.0);
                bool valid = true;
                for (const auto& obs : obstacles) {
                    if (calculateDistance(new_start, obs.position) < obs.radius + 10.0) {
                        valid = false;
                        break;
                    }
                }
                if (valid) {
                    std::cout << "[RRT] Using adjusted start position: (" << new_start.x << ", " << new_start.y << ")" << std::endl;
                    nodes.push_back(RRTNode(new_start, -1, 0.0));
                    break;
                }
            }
            if (nodes.empty()) {
                std::cout << "[RRT] Cannot find valid start position!" << std::endl;
                result.success = false;
                return result;
            }
        }
    }
    
    if (nodes.empty()) {
        nodes.push_back(RRTNode(start_pos, -1, 0.0));
    }
    
    int goal_node_index = -1;
    const double goal_threshold = 20.0; // Slightly larger threshold for easier goal reaching
    
    std::cout << "[RRT] Starting RRT path finding..." << std::endl;
    std::cout << "  Start: (" << start_pos.x << ", " << start_pos.y << ")" << std::endl;
    std::cout << "  Goal: (" << target_pos.x << ", " << target_pos.y << ")" << std::endl;
    std::cout << "  Obstacles: " << obstacles.size() << std::endl;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Vector2D random_point;
        
        // Goal biasing: sometimes aim directly for the goal
        if ((double(rand()) / RAND_MAX) < goal_bias_) {
            random_point = target_pos;
        } else {
            random_point = generateRandomPoint(min_bounds, max_bounds);
        }
        
        // Find nearest node in tree
        int nearest_index = findNearestNode(nodes, random_point);
        if (nearest_index == -1) continue;
        
        // Steer towards random point
        Vector2D new_position = steerTowards(nodes[nearest_index].position, random_point, rrt_step_size_);
        
        // Check if path from nearest to new position is collision-free
        if (!isCollisionFree(nodes[nearest_index].position, new_position, obstacles, 8.0)) {
            continue;
        }
        
        // Add new node to tree
        double new_cost = nodes[nearest_index].cost + calculateDistance(nodes[nearest_index].position, new_position);
        nodes.push_back(RRTNode(new_position, nearest_index, new_cost));
        int new_node_index = nodes.size() - 1;
        
        // Check if we reached the goal
        if (calculateDistance(new_position, target_pos) <= goal_threshold) {
            // Add exact target position as final node
            double final_cost = new_cost + calculateDistance(new_position, target_pos);
            nodes.push_back(RRTNode(target_pos, new_node_index, final_cost));
            goal_node_index = nodes.size() - 1;
            std::cout << "[RRT] Goal reached in " << iter << " iterations!" << std::endl;
            break;
        }
        
        // Progress indicator
        if (iter % 200 == 0) {
            std::cout << "[RRT] Iteration " << iter << ", nodes: " << nodes.size() << std::endl;
        }
    }
    
    if (goal_node_index != -1) {
        // Extract path
        std::vector<Vector2D> path_points = extractPath(nodes, goal_node_index);
        
        // Convert to trajectory
        double current_time = 0.0;
        double total_distance = 0.0;
        
        for (size_t i = 0; i < path_points.size(); ++i) {
            Vector2D velocity(0, 0);
            
            if (i < path_points.size() - 1) {
                Vector2D direction = path_points[i + 1] - path_points[i];
                double segment_distance = direction.length();
                total_distance += segment_distance;
                
                if (segment_distance > 0.001) {
                    velocity = direction.normalize() * max_speed;
                }
            }
            
            result.trajectory.push_back(TrajectoryPoint(path_points[i], velocity, current_time));
            current_time += time_step_;
        }
        
        result.success = true;
        result.total_distance = total_distance;
        result.total_time = current_time;
        
        std::cout << "[RRT] Path found! Distance: " << total_distance << ", Points: " << path_points.size() << std::endl;
    } else {
        std::cout << "[RRT] Failed to find path to goal" << std::endl;
        result.success = false;
    }
    
    return result;
}

PathTrajectory::PathResult PathTrajectory::findRRTStarPath(const Vector2D& start_pos,
                                                          const Vector2D& target_pos,
                                                          const std::vector<Obstacle>& obstacles,
                                                          double max_speed,
                                                          int max_iterations) {
    // RRT* implementation with path optimization
    PathResult result = findRRTPath(start_pos, target_pos, obstacles, max_speed, max_iterations);
    
    // Additional optimization can be added here
    if (result.success && result.trajectory.size() > 2) {
        result.trajectory = smoothTrajectory(result.trajectory);
    }
    
    return result;
}

Vector2D PathTrajectory::calculateAttractiveForce(const Vector2D& current_pos, const Vector2D& target_pos) {
    Vector2D direction = target_pos - current_pos;
    double distance = direction.length();
    
    if (distance < 0.001) {
        return Vector2D(0, 0);
    }
    
    // Linear attractive force
    return direction.normalize() * goal_weight_;
}

Vector2D PathTrajectory::calculateRepulsiveForce(const Vector2D& current_pos, const Obstacle& obstacle) {
    Vector2D direction = current_pos - obstacle.position;
    double distance = direction.length();
    
    if (distance < 0.001) {
        // Very close or at obstacle center, push away randomly
        return Vector2D(1, 0) * obstacle_weight_;
    }
    
    double influence_radius = obstacle.radius + 30.0; // Extended influence
    
    if (distance > influence_radius) {
        return Vector2D(0, 0); // No repulsive force beyond influence radius
    }
    
    // Inverse square repulsive force
    double force_magnitude = obstacle_weight_ / (distance * distance);
    
    // Consider obstacle velocity for dynamic avoidance
    Vector2D relative_vel = obstacle.velocity;
    if (relative_vel.length() > 0.1) {
        // Predict obstacle future position
        Vector2D future_obstacle_pos = obstacle.position + relative_vel * 1.0; // 1 second prediction
        Vector2D future_direction = current_pos - future_obstacle_pos;
        double future_distance = future_direction.length();
        
        if (future_distance < distance) {
            // Obstacle is approaching, increase repulsive force
            force_magnitude *= 2.0;
        }
    }
    
    return direction.normalize() * force_magnitude;
}

Vector2D PathTrajectory::calculateTotalForce(const Vector2D& current_pos, const Vector2D& target_pos, 
                                            const std::vector<Obstacle>& obstacles) {
    
    Vector2D attractive_force = calculateAttractiveForce(current_pos, target_pos);
    Vector2D repulsive_force(0, 0);
    
    for (const auto& obstacle : obstacles) {
        repulsive_force = repulsive_force + calculateRepulsiveForce(current_pos, obstacle);
    }
    
    return attractive_force + repulsive_force;
}

bool PathTrajectory::isCollisionFree(const Vector2D& pos1, const Vector2D& pos2, 
                                    const std::vector<Obstacle>& obstacles, double agent_radius) {
    
    Vector2D direction = pos2 - pos1;
    double total_distance = direction.length();
    
    if (total_distance < 0.001) {
        return true; // Same position
    }
    
    direction = direction.normalize();
    
    // Check collision along the path
    for (double d = 0; d <= total_distance; d += 5.0) { // Check every 5 units
        Vector2D test_pos = pos1 + direction * d;
        
        for (const auto& obstacle : obstacles) {
            if (calculateDistance(test_pos, obstacle.position) < obstacle.radius + agent_radius) {
                return false;
            }
        }
    }
    
    return true;
}

bool PathTrajectory::willCollide(const Vector2D& agent_pos, const Vector2D& agent_vel,
                                const Obstacle& obstacle, double time_horizon, double agent_radius) {
    
    // Predict positions over time horizon
    for (double t = 0; t <= time_horizon; t += time_step_) {
        Vector2D future_agent_pos = agent_pos + agent_vel * t;
        Vector2D future_obstacle_pos = obstacle.position + obstacle.velocity * t;
        
        if (calculateDistance(future_agent_pos, future_obstacle_pos) < obstacle.radius + agent_radius) {
            return true;
        }
    }
    
    return false;
}

std::vector<TrajectoryPoint> PathTrajectory::smoothTrajectory(const std::vector<TrajectoryPoint>& raw_trajectory) {
    if (raw_trajectory.size() < 3) {
        return raw_trajectory;
    }
    
    std::vector<TrajectoryPoint> smoothed;
    smoothed.push_back(raw_trajectory[0]); // Keep first point
    
    // Simple moving average smoothing
    for (size_t i = 1; i < raw_trajectory.size() - 1; ++i) {
        Vector2D avg_pos = (raw_trajectory[i-1].position + raw_trajectory[i].position + raw_trajectory[i+1].position) * (1.0/3.0);
        Vector2D avg_vel = (raw_trajectory[i-1].velocity + raw_trajectory[i].velocity + raw_trajectory[i+1].velocity) * (1.0/3.0);
        
        smoothed.push_back(TrajectoryPoint(avg_pos, avg_vel, raw_trajectory[i].time));
    }
    
    smoothed.push_back(raw_trajectory.back()); // Keep last point
    
    return smoothed;
}

double PathTrajectory::angleBetween(const Vector2D& v1, const Vector2D& v2) {
    double dot_product = v1.dot(v2);
    double magnitudes = v1.length() * v2.length();
    
    if (magnitudes < 0.001) {
        return 0.0;
    }
    
    double cos_angle = std::clamp(dot_product / magnitudes, -1.0, 1.0);
    return std::acos(cos_angle);
}

Vector2D PathTrajectory::limitVector(const Vector2D& vec, double max_magnitude) {
    double magnitude = vec.length();
    if (magnitude > max_magnitude) {
        return vec.normalize() * max_magnitude;
    }
    return vec;
}

double PathTrajectory::calculateDistance(const Vector2D& p1, const Vector2D& p2) {
    return (p2 - p1).length();
}

// RRT Helper Functions
Vector2D PathTrajectory::generateRandomPoint(const Vector2D& min_bounds, const Vector2D& max_bounds) {
    double x = min_bounds.x + (double(rand()) / RAND_MAX) * (max_bounds.x - min_bounds.x);
    double y = min_bounds.y + (double(rand()) / RAND_MAX) * (max_bounds.y - min_bounds.y);
    return Vector2D(x, y);
}

int PathTrajectory::findNearestNode(const std::vector<RRTNode>& nodes, const Vector2D& point) {
    if (nodes.empty()) return -1;
    
    int nearest_index = 0;
    double min_distance = calculateDistance(nodes[0].position, point);
    
    for (size_t i = 1; i < nodes.size(); ++i) {
        double distance = calculateDistance(nodes[i].position, point);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_index = i;
        }
    }
    
    return nearest_index;
}

Vector2D PathTrajectory::steerTowards(const Vector2D& from, const Vector2D& to, double step_size) {
    Vector2D direction = to - from;
    double distance = direction.length();
    
    if (distance <= step_size) {
        return to;
    }
    
    return from + direction.normalize() * step_size;
}

std::vector<int> PathTrajectory::findNearbyNodes(const std::vector<RRTNode>& nodes, const Vector2D& point, double radius) {
    std::vector<int> nearby_indices;
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (calculateDistance(nodes[i].position, point) <= radius) {
            nearby_indices.push_back(i);
        }
    }
    
    return nearby_indices;
}

std::vector<Vector2D> PathTrajectory::extractPath(const std::vector<RRTNode>& nodes, int goal_index) {
    std::vector<Vector2D> path;
    
    int current_index = goal_index;
    while (current_index != -1) {
        path.push_back(nodes[current_index].position);
        current_index = nodes[current_index].parent_index;
    }
    
    // Reverse path to get start -> goal order
    std::reverse(path.begin(), path.end());
    
    return path;
}

}