#include "ConfigManager.h"
#include "PathFinder.h"
#include <iostream>

/**
 * @brief Example showing how to use the modular configuration system
 * This demonstrates how easy it is to adjust pathfinding settings
 */

void demonstrateEasyConfiguration() {
    std::cout << "\n=== PATHFINDING CONFIGURATION EXAMPLES ===" << std::endl;
    
    // ============ METHOD 1: USE PRESETS ============
    std::cout << "\n1. Using Presets:" << std::endl;
    
    // Conservative settings - safer but slower
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::CONSERVATIVE);
    std::cout << "   CONSERVATIVE: Large safety margins, slower replanning" << std::endl;
    
    // Aggressive settings - faster but riskier  
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::AGGRESSIVE);
    std::cout << "   AGGRESSIVE: Small safety margins, fast replanning" << std::endl;
    
    // Debug settings - lots of logging
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::DEBUG);
    std::cout << "   DEBUG: Enabled logging for development" << std::endl;
    
    // Balanced settings (default)
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::BALANCED);
    std::cout << "   BALANCED: Good compromise (default)" << std::endl;
    
    // ============ METHOD 2: ADJUST SPECIFIC PARAMETERS ============
    std::cout << "\n2. Adjusting Specific Parameters:" << std::endl;
    
    // Make obstacle avoidance more aggressive (0.0 = very safe, 1.0 = very aggressive)
    ctrl::GlobalConfig().setObstacleAvoidanceLevel(0.8);
    std::cout << "   Set obstacle avoidance to 80% aggressive" << std::endl;
    
    // Increase replanning frequency for faster response
    ctrl::GlobalConfig().setReplanningFrequency(100); // 100 Hz instead of 50 Hz
    std::cout << "   Set replanning frequency to 100 Hz (every 10ms)" << std::endl;
    
    // Reduce collision safety margin for tighter paths
    ctrl::GlobalConfig().setCollisionMargin(250.0); // 250mm instead of 400mm
    std::cout << "   Set collision margin to 250mm" << std::endl;
    
    // Enable debug mode for development
    ctrl::GlobalConfig().setDebugMode(true);
    std::cout << "   Enabled debug logging" << std::endl;
    
    // ============ METHOD 3: QUICK ADJUSTMENTS ============
    std::cout << "\n3. Quick Adjustments:" << std::endl;
    
    // Make robot more conservative
    ctrl::GlobalConfig().makeMoreConservative();
    std::cout << "   Made robot more conservative" << std::endl;
    
    // Or make it more aggressive
    ctrl::GlobalConfig().makeMoreAggressive();  
    std::cout << "   Made robot more aggressive" << std::endl;
    
    // Enable performance optimizations
    ctrl::GlobalConfig().enablePerformanceOptimizations();
    std::cout << "   Enabled performance optimizations" << std::endl;
    
    // ============ METHOD 4: DIRECT CONFIGURATION ACCESS ============
    std::cout << "\n4. Direct Configuration Access:" << std::endl;
    
    // Get direct access to configuration for fine-tuning
    auto& config = ctrl::GlobalConfig().getConfig();
    
    // Fine-tune obstacle avoidance
    config.obstacle_avoidance.base_safety_margin_m = 0.12;
    config.obstacle_avoidance.extended_safety_margin_m = 0.20;
    config.obstacle_avoidance.max_waypoints = 10;
    
    // Fine-tune collision detection
    config.collision_detection.safety_margin_mm = 350.0;
    config.collision_detection.enable_adaptive_margins = true;
    config.collision_detection.adaptive_margin_multiplier = 1.5;
    
    // Fine-tune controller
    config.controller.replan_frequency_hz = 75.0;
    config.controller.collision_brake_time_s = 0.03;
    
    std::cout << "   Fine-tuned specific parameters directly" << std::endl;
    
    // ============ METHOD 5: USE WITH PATHFINDER ============
    std::cout << "\n5. Using Configuration with PathFinder:" << std::endl;
    
    // Create PathFinder with current global configuration
    ctrl::PathFinder pathfinder(ctrl::GlobalConfig().getConfig());
    std::cout << "   Created PathFinder with global configuration" << std::endl;
    
    // Or create PathFinder with specific configuration
    auto custom_config = ctrl::PathfindingConfig::createAggressive();
    custom_config.debug.enable_pathfinding_logs = true;
    ctrl::PathFinder custom_pathfinder(custom_config);
    std::cout << "   Created PathFinder with custom configuration" << std::endl;
    
    // ============ VIEW CURRENT SETTINGS ============
    std::cout << "\n6. Current Configuration:" << std::endl;
    ctrl::GlobalConfig().printConfig();
}

void demonstrateScenarioConfigurations() {
    std::cout << "\n=== SCENARIO-SPECIFIC CONFIGURATIONS ===" << std::endl;
    
    // ============ SCENARIO 1: DENSE OBSTACLE FIELD ============
    std::cout << "\nScenario 1: Dense Obstacle Field" << std::endl;
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::CONSERVATIVE);
    ctrl::GlobalConfig().setReplanningFrequency(75); // Higher frequency for dynamic obstacles
    ctrl::GlobalConfig().setCollisionMargin(450.0); // Larger margins for safety
    ctrl::GlobalConfig().enablePerformanceOptimizations(); // Need performance with many obstacles
    std::cout << "   Configured for dense obstacle field navigation" << std::endl;
    
    // ============ SCENARIO 2: HIGH-SPEED NAVIGATION ============
    std::cout << "\nScenario 2: High-Speed Navigation" << std::endl;
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::AGGRESSIVE);
    ctrl::GlobalConfig().setReplanningFrequency(120); // Very high frequency
    ctrl::GlobalConfig().setObstacleAvoidanceLevel(0.9); // Very aggressive
    std::cout << "   Configured for high-speed navigation" << std::endl;
    
    // ============ SCENARIO 3: DEVELOPMENT/DEBUGGING ============
    std::cout << "\nScenario 3: Development/Debugging" << std::endl;
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::DEBUG);
    ctrl::GlobalConfig().setReplanningFrequency(20); // Slower for easier debugging
    ctrl::GlobalConfig().disablePerformanceOptimizations(); // Full checking for debugging
    std::cout << "   Configured for development and debugging" << std::endl;
    
    // ============ SCENARIO 4: SOCCER MATCH ============
    std::cout << "\nScenario 4: Soccer Match" << std::endl;
    ctrl::GlobalConfig().usePreset(ctrl::ConfigManager::Preset::BALANCED);
    ctrl::GlobalConfig().setReplanningFrequency(50); // Standard 50Hz like real Advanced
    ctrl::GlobalConfig().setObstacleAvoidanceLevel(0.6); // Balanced aggressiveness
    ctrl::GlobalConfig().enablePerformanceOptimizations(); // Need performance in real-time
    std::cout << "   Configured for soccer match conditions" << std::endl;
}

int main() {
    demonstrateEasyConfiguration();
    demonstrateScenarioConfigurations();
    
    std::cout << "\n=== CONFIGURATION SYSTEM READY ===" << std::endl;
    std::cout << "The modular configuration system is now available!" << std::endl;
    std::cout << "You can easily adjust:" << std::endl;
    std::cout << "  - Obstacle avoidance aggressiveness" << std::endl;
    std::cout << "  - Collision detection margins" << std::endl;
    std::cout << "  - Controller replanning frequency" << std::endl;
    std::cout << "  - Debug logging levels" << std::endl;
    std::cout << "  - Performance optimizations" << std::endl;
    std::cout << "  - And much more!" << std::endl;
    
    return 0;
}