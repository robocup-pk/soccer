#include <iostream>
#include <cmath>
#include <Eigen/Dense>

void testFrameConversion() {
    std::cout << "Testing World to Body Frame Conversion\n";
    std::cout << "======================================\n\n";
    
    // Test Case 1: Robot facing right (θ = 0)
    {
        double theta = 0.0;
        double world_vx = 1.0;  // Moving right in world
        double world_vy = 0.0;
        
        double body_vx = world_vx * std::cos(theta) + world_vy * std::sin(theta);
        double body_vy = -world_vx * std::sin(theta) + world_vy * std::cos(theta);
        
        std::cout << "Test 1: Robot facing right (θ=0°)\n";
        std::cout << "  World velocity: (" << world_vx << ", " << world_vy << ")\n";
        std::cout << "  Body velocity:  (" << body_vx << ", " << body_vy << ")\n";
        std::cout << "  Expected: (1, 0) - robot moves forward\n";
        std::cout << "  Result: " << (std::abs(body_vx - 1.0) < 0.001 && std::abs(body_vy) < 0.001 ? "PASS" : "FAIL") << "\n\n";
    }
    
    // Test Case 2: Robot facing up (θ = π/2)
    {
        double theta = M_PI/2;
        double world_vx = 1.0;  // Moving right in world
        double world_vy = 0.0;
        
        double body_vx = world_vx * std::cos(theta) + world_vy * std::sin(theta);
        double body_vy = -world_vx * std::sin(theta) + world_vy * std::cos(theta);
        
        std::cout << "Test 2: Robot facing up (θ=90°)\n";
        std::cout << "  World velocity: (" << world_vx << ", " << world_vy << ")\n";
        std::cout << "  Body velocity:  (" << body_vx << ", " << body_vy << ")\n";
        std::cout << "  Expected: (0, -1) - robot moves to its left\n";
        std::cout << "  Result: " << (std::abs(body_vx) < 0.001 && std::abs(body_vy + 1.0) < 0.001 ? "PASS" : "FAIL") << "\n\n";
    }
    
    // Test Case 3: Robot facing left (θ = π)
    {
        double theta = M_PI;
        double world_vx = 1.0;  // Moving right in world
        double world_vy = 0.0;
        
        double body_vx = world_vx * std::cos(theta) + world_vy * std::sin(theta);
        double body_vy = -world_vx * std::sin(theta) + world_vy * std::cos(theta);
        
        std::cout << "Test 3: Robot facing left (θ=180°)\n";
        std::cout << "  World velocity: (" << world_vx << ", " << world_vy << ")\n";
        std::cout << "  Body velocity:  (" << body_vx << ", " << body_vy << ")\n";
        std::cout << "  Expected: (-1, 0) - robot moves backward\n";
        std::cout << "  Result: " << (std::abs(body_vx + 1.0) < 0.001 && std::abs(body_vy) < 0.001 ? "PASS" : "FAIL") << "\n\n";
    }
    
    // Test Case 4: Robot at 45 degrees moving diagonally
    {
        double theta = M_PI/4;
        double world_vx = 1.0/std::sqrt(2);  // Moving diagonally
        double world_vy = 1.0/std::sqrt(2);
        
        double body_vx = world_vx * std::cos(theta) + world_vy * std::sin(theta);
        double body_vy = -world_vx * std::sin(theta) + world_vy * std::cos(theta);
        
        std::cout << "Test 4: Robot facing 45° with diagonal world velocity\n";
        std::cout << "  World velocity: (" << world_vx << ", " << world_vy << ")\n";
        std::cout << "  Body velocity:  (" << body_vx << ", " << body_vy << ")\n";
        std::cout << "  Expected: (1, 0) - aligned motion becomes forward\n";
        std::cout << "  Result: " << (std::abs(body_vx - 1.0) < 0.001 && std::abs(body_vy) < 0.001 ? "PASS" : "FAIL") << "\n\n";
    }
    
    // Inverse verification
    std::cout << "Inverse Transformation Test:\n";
    std::cout << "----------------------------\n";
    {
        double theta = M_PI/3;  // 60 degrees
        double world_vx = 0.5;
        double world_vy = 0.866;  // ~sqrt(3)/2
        
        // World to Body
        double body_vx = world_vx * std::cos(theta) + world_vy * std::sin(theta);
        double body_vy = -world_vx * std::sin(theta) + world_vy * std::cos(theta);
        
        // Body back to World (inverse)
        double world_vx_check = body_vx * std::cos(theta) - body_vy * std::sin(theta);
        double world_vy_check = body_vx * std::sin(theta) + body_vy * std::cos(theta);
        
        std::cout << "  Original world velocity: (" << world_vx << ", " << world_vy << ")\n";
        std::cout << "  Body velocity: (" << body_vx << ", " << body_vy << ")\n";
        std::cout << "  Recovered world velocity: (" << world_vx_check << ", " << world_vy_check << ")\n";
        
        bool inverse_correct = (std::abs(world_vx - world_vx_check) < 0.001 && 
                                std::abs(world_vy - world_vy_check) < 0.001);
        std::cout << "  Inverse transformation: " << (inverse_correct ? "PASS" : "FAIL") << "\n\n";
    }
}

int main() {
    testFrameConversion();
    
    std::cout << "\nConversion formulas used:\n";
    std::cout << "  body_vx = world_vx * cos(θ) + world_vy * sin(θ)\n";
    std::cout << "  body_vy = -world_vx * sin(θ) + world_vy * cos(θ)\n";
    std::cout << "\nWhere θ is the robot's orientation in the world frame.\n";
    
    return 0;
}