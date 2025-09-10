#pragma once

#include <vector>
#include <string>

namespace ctrl {

/**
 * @brief Centralized configuration for all pathfinding and motion planning settings
 * Makes it easy to adjust obstacle avoidance, controllers, collision checking, etc.
 */
struct PathfindingConfig {
    
    // ============ OBSTACLE AVOIDANCE SETTINGS ============
    struct ObstacleAvoidance {
        // Waypoint generation
        double base_safety_margin_m = 0.15;        // Base safety margin around obstacles
        double extended_safety_margin_m = 0.25;    // Extended margin for backup waypoints
        double min_clearance_check_m = 0.1;        // Minimum clearance when checking waypoints
        
        // Waypoint offsets (multipliers of obstacle radius)
        std::vector<double> offset_multipliers = {1.5, -1.5, 2.0, -2.0, 2.5, -2.5};
        
        // Path positions to try waypoints
        std::vector<double> path_fractions = {0.25, 0.5, 0.75};
        
        // Fallback circular pattern
        double circular_pattern_radius_multiplier = 2.0;  // Multiplier of max obstacle radius
        std::vector<double> circular_angles = {0.785, -0.785, 2.356, -2.356}; // 45°, -45°, 135°, -135°
        
        // Maximum waypoints to generate
        int max_waypoints = 8;
        
        // Enable/disable features
        bool enable_adaptive_offsets = true;        // Adapt offsets to obstacle sizes
        bool enable_circular_fallback = true;      // Use circular pattern when linear fails
        bool enable_multi_segment_paths = true;    // Create paths through multiple waypoints
    } obstacle_avoidance;
    
    // ============ COLLISION DETECTION SETTINGS ============
    struct CollisionDetection {
        // Line-to-circle collision checking
        double safety_margin_mm = 400.0;           // Safety margin for collision detection
        bool enable_adaptive_margins = true;       // Adapt margins to obstacle sizes
        double adaptive_margin_multiplier = 1.2;   // Multiplier for adaptive margins
        
        // Path validation
        bool check_waypoint_to_destination = true; // Check full path including waypoint->destination
        bool enable_multi_obstacle_check = true;   // Check against all obstacles
        
        // Performance settings
        bool enable_distance_optimizations = true; // Skip far obstacles
        double max_check_distance_m = 5.0;         // Maximum distance to check obstacles
    } collision_detection;
    
    // ============ CONTROLLER SETTINGS ============
    struct Controller {
        // Replanning frequency
        double replan_frequency_hz = 50.0;         // How often to replan trajectories
        double min_replanning_interval_s = 0.02;   // Minimum time between replans
        
        // Collision response
        double collision_brake_time_s = 0.05;      // Time horizon for collision detection
        double brake_velocity_tolerance = 0.6;     // Velocity buffer for braking
        double max_position_error_m = 0.3;         // Position error threshold
        
        // Velocity limiting
        double velocity_limiter_min = 0.5;         // Minimum velocity when problems detected
        double velocity_increase_rate = 5.0;       // Rate to increase velocity when clear
        double velocity_decrease_rate = -5.0;      // Rate to decrease velocity when problems
        
        // Enable/disable features
        bool enable_replanning = true;             // Enable dynamic replanning
        bool enable_collision_braking = true;      // Enable collision avoidance braking
        bool enable_velocity_limiting = true;      // Enable adaptive velocity limiting
    } controller;
    
    // ============ TRAJECTORY GENERATION SETTINGS ============
    struct TrajectoryGeneration {
        // Path smoothing
        double waypoint_connection_fraction = 0.6; // Where to connect trajectory segments
        bool enable_smooth_paths = true;           // Use smooth multi-waypoint paths
        bool enable_velocity_continuity = true;    // Maintain velocity continuity at waypoints
        
        // Trajectory timing
        double min_trajectory_time_s = 0.1;        // Minimum trajectory duration
        double max_trajectory_time_s = 10.0;       // Maximum trajectory duration
        
        // Motion constraints (can be overridden)
        double default_max_velocity_mps = 3.0;     // Default max velocity
        double default_max_acceleration_mps2 = 3.0; // Default max acceleration
        double default_max_angular_velocity_radps = 10.0; // Default max angular velocity
        double default_max_angular_acceleration_radps2 = 30.0; // Default max angular acceleration
    } trajectory_generation;
    
    // ============ DEBUG AND LOGGING SETTINGS ============
    struct Debug {
        bool enable_pathfinding_logs = true;       // Log pathfinding decisions
        bool enable_collision_logs = false;        // Log collision detection details
        bool enable_waypoint_logs = true;          // Log waypoint generation
        bool enable_performance_logs = false;      // Log performance metrics
        
        // Visualization settings
        bool show_waypoints = true;                // Show generated waypoints
        bool show_obstacle_margins = false;       // Show obstacle safety margins
        bool show_collision_checks = false;       // Show collision check results
    } debug;
    
    // ============ PRESET CONFIGURATIONS ============
    
    /**
     * @brief Conservative settings - safer but slower paths
     */
    static PathfindingConfig createConservative() {
        PathfindingConfig config;
        config.obstacle_avoidance.base_safety_margin_m = 0.25;
        config.obstacle_avoidance.extended_safety_margin_m = 0.4;
        config.collision_detection.safety_margin_mm = 500.0;
        config.controller.replan_frequency_hz = 30.0;
        return config;
    }
    
    /**
     * @brief Aggressive settings - faster but closer to obstacles
     */
    static PathfindingConfig createAggressive() {
        PathfindingConfig config;
        config.obstacle_avoidance.base_safety_margin_m = 0.1;
        config.obstacle_avoidance.extended_safety_margin_m = 0.15;
        config.collision_detection.safety_margin_mm = 300.0;
        config.controller.replan_frequency_hz = 100.0;
        return config;
    }
    
    /**
     * @brief Balanced settings - good compromise (default)
     */
    static PathfindingConfig createBalanced() {
        return PathfindingConfig(); // Use default values
    }
    
    /**
     * @brief Debug settings - lots of logging for development
     */
    static PathfindingConfig createDebug() {
        PathfindingConfig config;
        config.debug.enable_pathfinding_logs = true;
        config.debug.enable_collision_logs = true;
        config.debug.enable_waypoint_logs = true;
        config.debug.enable_performance_logs = true;
        config.debug.show_waypoints = true;
        config.debug.show_obstacle_margins = true;
        return config;
    }
};

} // namespace ctrl