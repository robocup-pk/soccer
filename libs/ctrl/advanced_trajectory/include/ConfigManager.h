#pragma once

#include "PathfindingConfig.h"
#include <string>
#include <map>
#include <memory>

namespace ctrl {

/**
 * @brief Easy-to-use configuration manager for pathfinding settings
 * Provides simple interface to change obstacle avoidance, controllers, collision detection, etc.
 */
class ConfigManager {
public:
    enum class Preset {
        BALANCED,      // Good balance of speed and safety (default)
        CONSERVATIVE,  // Safer, larger margins, slower replanning
        AGGRESSIVE,    // Faster, smaller margins, higher replanning frequency
        DEBUG          // Lots of logging for development
    };
    
    ConfigManager() : current_config_(PathfindingConfig::createBalanced()) {}
    
    // ============ EASY PRESET SWITCHING ============
    
    /**
     * @brief Switch to a preset configuration
     */
    void usePreset(Preset preset) {
        switch (preset) {
            case Preset::BALANCED:
                current_config_ = PathfindingConfig::createBalanced();
                break;
            case Preset::CONSERVATIVE:
                current_config_ = PathfindingConfig::createConservative();
                break;
            case Preset::AGGRESSIVE:
                current_config_ = PathfindingConfig::createAggressive();
                break;
            case Preset::DEBUG:
                current_config_ = PathfindingConfig::createDebug();
                break;
        }
    }
    
    // ============ EASY PARAMETER ADJUSTMENT ============
    
    /**
     * @brief Adjust obstacle avoidance aggressiveness (0.0 = very safe, 1.0 = very aggressive)
     */
    void setObstacleAvoidanceLevel(double level) {
        level = std::max(0.0, std::min(1.0, level)); // Clamp to [0,1]
        
        // Interpolate between conservative and aggressive settings
        double conservative_margin = 0.25;
        double aggressive_margin = 0.1;
        current_config_.obstacle_avoidance.base_safety_margin_m = 
            conservative_margin + level * (aggressive_margin - conservative_margin);
            
        double conservative_collision_margin = 500.0;
        double aggressive_collision_margin = 300.0;
        current_config_.collision_detection.safety_margin_mm = 
            conservative_collision_margin + level * (aggressive_collision_margin - conservative_collision_margin);
    }
    
    /**
     * @brief Set replanning frequency in Hz
     */
    void setReplanningFrequency(double hz) {
        current_config_.controller.replan_frequency_hz = hz;
        current_config_.controller.min_replanning_interval_s = 1.0 / hz;
    }
    
    /**
     * @brief Enable/disable debug logging
     */
    void setDebugMode(bool enable) {
        current_config_.debug.enable_pathfinding_logs = enable;
        current_config_.debug.enable_waypoint_logs = enable;
        current_config_.debug.enable_collision_logs = enable;
        current_config_.debug.show_waypoints = enable;
    }
    
    /**
     * @brief Set collision safety margin in millimeters
     */
    void setCollisionMargin(double margin_mm) {
        current_config_.collision_detection.safety_margin_mm = margin_mm;
    }
    
    /**
     * @brief Enable/disable adaptive obstacle margins
     */
    void setAdaptiveMargins(bool enable) {
        current_config_.collision_detection.enable_adaptive_margins = enable;
        current_config_.obstacle_avoidance.enable_adaptive_offsets = enable;
    }
    
    // ============ QUICK ACCESS METHODS ============
    
    /**
     * @brief Make robot more conservative (safer, slower)
     */
    void makeMoreConservative() {
        current_config_.obstacle_avoidance.base_safety_margin_m *= 1.2;
        current_config_.collision_detection.safety_margin_mm *= 1.2;
        current_config_.controller.replan_frequency_hz *= 0.8;
    }
    
    /**
     * @brief Make robot more aggressive (faster, riskier)
     */
    void makeMoreAggressive() {
        current_config_.obstacle_avoidance.base_safety_margin_m *= 0.8;
        current_config_.collision_detection.safety_margin_mm *= 0.8;
        current_config_.controller.replan_frequency_hz *= 1.2;
    }
    
    /**
     * @brief Enable performance optimizations
     */
    void enablePerformanceOptimizations() {
        current_config_.collision_detection.enable_distance_optimizations = true;
        current_config_.collision_detection.max_check_distance_m = 3.0;
        current_config_.obstacle_avoidance.max_waypoints = 6;
    }
    
    /**
     * @brief Disable performance optimizations (more thorough checking)
     */
    void disablePerformanceOptimizations() {
        current_config_.collision_detection.enable_distance_optimizations = false;
        current_config_.collision_detection.max_check_distance_m = 10.0;
        current_config_.obstacle_avoidance.max_waypoints = 12;
    }
    
    // ============ CONFIGURATION ACCESS ============
    
    const PathfindingConfig& getConfig() const { return current_config_; }
    PathfindingConfig& getConfig() { return current_config_; }
    
    /**
     * @brief Print current configuration summary
     */
    void printConfig() const {
        std::cout << "\n=== PATHFINDING CONFIGURATION ===" << std::endl;
        std::cout << "Obstacle Avoidance:" << std::endl;
        std::cout << "  Safety margin: " << current_config_.obstacle_avoidance.base_safety_margin_m << "m" << std::endl;
        std::cout << "  Adaptive offsets: " << (current_config_.obstacle_avoidance.enable_adaptive_offsets ? "ON" : "OFF") << std::endl;
        std::cout << "  Max waypoints: " << current_config_.obstacle_avoidance.max_waypoints << std::endl;
        
        std::cout << "Collision Detection:" << std::endl;
        std::cout << "  Safety margin: " << current_config_.collision_detection.safety_margin_mm << "mm" << std::endl;
        std::cout << "  Adaptive margins: " << (current_config_.collision_detection.enable_adaptive_margins ? "ON" : "OFF") << std::endl;
        std::cout << "  Distance optimizations: " << (current_config_.collision_detection.enable_distance_optimizations ? "ON" : "OFF") << std::endl;
        
        std::cout << "Controller:" << std::endl;
        std::cout << "  Replan frequency: " << current_config_.controller.replan_frequency_hz << " Hz" << std::endl;
        std::cout << "  Collision brake time: " << current_config_.controller.collision_brake_time_s << "s" << std::endl;
        
        std::cout << "Debug:" << std::endl;
        std::cout << "  Pathfinding logs: " << (current_config_.debug.enable_pathfinding_logs ? "ON" : "OFF") << std::endl;
        std::cout << "  Waypoint logs: " << (current_config_.debug.enable_waypoint_logs ? "ON" : "OFF") << std::endl;
        std::cout << "  Collision logs: " << (current_config_.debug.enable_collision_logs ? "ON" : "OFF") << std::endl;
        std::cout << "=================================" << std::endl;
    }
    
private:
    PathfindingConfig current_config_;
};

// ============ GLOBAL CONFIGURATION INSTANCE ============

/**
 * @brief Global configuration manager - easy access from anywhere
 * 
 * Usage examples:
 * 
 *   // Use preset configurations
 *   GlobalConfig().usePreset(ConfigManager::Preset::AGGRESSIVE);
 * 
 *   // Adjust specific parameters  
 *   GlobalConfig().setObstacleAvoidanceLevel(0.8); // 80% aggressive
 *   GlobalConfig().setReplanningFrequency(100);    // 100 Hz
 *   GlobalConfig().setDebugMode(true);             // Enable logging
 * 
 *   // Quick adjustments
 *   GlobalConfig().makeMoreConservative();
 *   GlobalConfig().enablePerformanceOptimizations();
 * 
 *   // Print current settings
 *   GlobalConfig().printConfig();
 */
inline ConfigManager& GlobalConfig() {
    static ConfigManager instance;
    return instance;
}

} // namespace ctrl