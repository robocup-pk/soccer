#pragma once

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

namespace ctrl {

/**
 * @brief Generates random intermediate destinations for path finding
 */
class SubDestinationGenerator {
public:
    SubDestinationGenerator() : rng_(std::random_device{}()) {}
    
    /**
     * @brief Generate normalized sub-destinations (random angles and scales)
     * @param num_destinations Number of sub-destinations to generate
     * @return List of normalized sub-destinations
     */
    std::vector<Eigen::Vector2d> generateNormalizedSubDestinations(int num_destinations = 5) {
        std::vector<Eigen::Vector2d> destinations;
        
        // Add last successful sub-destination if available
        if (last_normalized_subdest_.has_value()) {
            destinations.push_back(last_normalized_subdest_.value());
        }
        
        // Generate random sub-destinations
        for (int i = 0; i < num_destinations; ++i) {
            double angle_range = MAX_ANGLE - MIN_ANGLE;
            std::uniform_real_distribution<double> angle_dist(-angle_range, angle_range);
            double angle = angle_dist(rng_);
            angle += (angle > 0 ? 1 : -1) * MIN_ANGLE; // Add minimum angle offset
            
            std::uniform_real_distribution<double> scale_dist(MIN_SCALE, MAX_SCALE);
            double scale = scale_dist(rng_);
            
            // Create vector from angle and length
            Eigen::Vector2d subdest(cos(angle) * scale, sin(angle) * scale);
            destinations.push_back(subdest);
        }
        
        // Sort by angle magnitude
        std::sort(destinations.begin(), destinations.end(),
            [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                return std::abs(atan2(a.y(), a.x())) < std::abs(atan2(b.y(), b.x()));
            });
        
        return destinations;
    }
    
    /**
     * @brief Generate actual sub-destinations for path finding
     * @param start Start position
     * @param dest Final destination
     * @return List of intermediate destinations to try
     */
    std::vector<Eigen::Vector2d> generateSubDestinations(
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& dest) {
        
        Eigen::Vector2d start_to_dest = dest - start;
        double distance = start_to_dest.norm();
        double direction = atan2(start_to_dest.y(), start_to_dest.x());
        
        auto normalized = generateNormalizedSubDestinations();
        std::vector<Eigen::Vector2d> subdestinations;
        
        // BOUNDARY LIMITS (prevent robots from going too far)
        const double MAX_DETOUR_DISTANCE = std::min(2.0, distance * 1.5); // Max 2m or 1.5x direct distance
        const double FIELD_BOUNDARY = 2.5; // Don't go beyond ±2.5m from origin
        
        for (const auto& norm_dest : normalized) {
            // Rotate by direction and scale by distance
            double rotated_angle = atan2(norm_dest.y(), norm_dest.x()) + direction;
            double scaled_length = std::min(norm_dest.norm() * distance, MAX_DETOUR_DISTANCE);
            
            Eigen::Vector2d actual_dest(
                start.x() + cos(rotated_angle) * scaled_length,
                start.y() + sin(rotated_angle) * scaled_length
            );
            
            // BOUNDARY CHECK: Skip sub-destinations that go too far out of bounds
            if (std::abs(actual_dest.x()) > FIELD_BOUNDARY || 
                std::abs(actual_dest.y()) > FIELD_BOUNDARY) {
                continue; // Skip this sub-destination
            }
            
            // Also check if sub-destination is reasonable relative to direct path
            double detour_ratio = (actual_dest - start).norm() / distance;
            if (detour_ratio > 2.0) {
                continue; // Skip sub-destinations that are more than 2x longer
            }
            
            subdestinations.push_back(actual_dest);
        }
        
        return subdestinations;
    }
    
    /**
     * @brief Store last successful sub-destination for reuse
     */
    void setLastSuccessful(const Eigen::Vector2d& normalized_subdest) {
        last_normalized_subdest_ = normalized_subdest;
    }
    
private:
    // OPTIMIZED constants for shorter, more reasonable paths
    static constexpr double MIN_ANGLE = 15.0 * M_PI / 180.0;  // 15 degrees in radians
    static constexpr double MAX_ANGLE = 75.0 * M_PI / 180.0;  // 75 degrees in radians (reduced from 140°)
    static constexpr double MIN_SCALE = 0.3;  // Reduced from 0.5 for shorter detours
    static constexpr double MAX_SCALE = 0.8;  // Reduced from 1.5 for shorter detours
    
    std::mt19937 rng_;
    std::optional<Eigen::Vector2d> last_normalized_subdest_;
};

} // namespace ctrl