#ifndef PATH_TRAJECTORY_H
#define PATH_TRAJECTORY_H

#include <vector>
#include <cmath>
#include "Vector2D.h"

namespace algos {

struct Obstacle {
    Vector2D position;
    double radius;
    Vector2D velocity;
    
    Obstacle(const Vector2D& pos, double r, const Vector2D& vel = Vector2D(0, 0)) 
        : position(pos), radius(r), velocity(vel) {}
};

struct TrajectoryPoint {
    Vector2D position;
    Vector2D velocity;
    double time;
    
    TrajectoryPoint(const Vector2D& pos, const Vector2D& vel, double t) 
        : position(pos), velocity(vel), time(t) {}
};

struct RRTNode {
    Vector2D position;
    int parent_index;
    double cost;
    
    RRTNode(const Vector2D& pos, int parent = -1, double c = 0.0) 
        : position(pos), parent_index(parent), cost(c) {}
};

class PathTrajectory {
public:
    struct PathResult {
        std::vector<TrajectoryPoint> trajectory;
        bool success;
        double total_time;
        double total_distance;
        
        PathResult() : success(false), total_time(0.0), total_distance(0.0) {}
    };
    
    PathTrajectory();
    ~PathTrajectory();
    
    // Main path finding method
    PathResult findPath(const Vector2D& start_pos, double start_angle,
                       const Vector2D& target_pos, double target_angle,
                       const std::vector<Obstacle>& obstacles,
                       double max_speed = 100.0, double max_acceleration = 50.0);
    
    // Simple direct path with obstacle avoidance
    PathResult findDirectPath(const Vector2D& start_pos, 
                             const Vector2D& target_pos,
                             const std::vector<Obstacle>& obstacles,
                             double max_speed = 100.0);
    
    // Potential field-based path finding
    PathResult findPotentialFieldPath(const Vector2D& start_pos,
                                     const Vector2D& target_pos,
                                     const std::vector<Obstacle>& obstacles,
                                     double max_speed = 100.0);
    
    // RRT-based path finding (best for obstacle avoidance)
    PathResult findRRTPath(const Vector2D& start_pos,
                          const Vector2D& target_pos,
                          const std::vector<Obstacle>& obstacles,
                          double max_speed = 100.0,
                          int max_iterations = 1000);
    
    // RRT* (optimized version)
    PathResult findRRTStarPath(const Vector2D& start_pos,
                              const Vector2D& target_pos,
                              const std::vector<Obstacle>& obstacles,
                              double max_speed = 100.0,
                              int max_iterations = 1000);
    
    // Configuration
    void setTimeStep(double dt) { time_step_ = dt; }
    void setLookaheadDistance(double distance) { lookahead_distance_ = distance; }
    void setObstacleAvoidanceWeight(double weight) { obstacle_weight_ = weight; }
    void setGoalAttractionWeight(double weight) { goal_weight_ = weight; }
    
private:
    double time_step_;
    double lookahead_distance_;
    double obstacle_weight_;
    double goal_weight_;
    
    // RRT parameters
    double rrt_step_size_;
    double goal_bias_;
    double rewire_radius_;
    
    // Physics calculations
    Vector2D calculateAttractiveForce(const Vector2D& current_pos, const Vector2D& target_pos);
    Vector2D calculateRepulsiveForce(const Vector2D& current_pos, const Obstacle& obstacle);
    Vector2D calculateTotalForce(const Vector2D& current_pos, const Vector2D& target_pos, 
                                const std::vector<Obstacle>& obstacles);
    
    // Collision detection
    bool isCollisionFree(const Vector2D& pos1, const Vector2D& pos2, 
                        const std::vector<Obstacle>& obstacles, double agent_radius = 10.0);
    bool willCollide(const Vector2D& agent_pos, const Vector2D& agent_vel,
                    const Obstacle& obstacle, double time_horizon = 2.0, 
                    double agent_radius = 10.0);
    
    // Trajectory smoothing
    std::vector<TrajectoryPoint> smoothTrajectory(const std::vector<TrajectoryPoint>& raw_trajectory);
    
    // RRT helper functions
    Vector2D generateRandomPoint(const Vector2D& min_bounds, const Vector2D& max_bounds);
    int findNearestNode(const std::vector<RRTNode>& nodes, const Vector2D& point);
    Vector2D steerTowards(const Vector2D& from, const Vector2D& to, double step_size);
    std::vector<int> findNearbyNodes(const std::vector<RRTNode>& nodes, const Vector2D& point, double radius);
    std::vector<Vector2D> extractPath(const std::vector<RRTNode>& nodes, int goal_index);
    
    // Utility functions
    double angleBetween(const Vector2D& v1, const Vector2D& v2);
    Vector2D limitVector(const Vector2D& vec, double max_magnitude);
    double calculateDistance(const Vector2D& p1, const Vector2D& p2);
};

}

#endif // PATH_TRAJECTORY_H