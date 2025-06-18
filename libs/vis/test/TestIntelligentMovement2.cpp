#include "IntelligentMovement2.h"
#include "GameObject.h"
#include "ResourceManager.h"
#include <iostream>
#include <map>

int main() {
    std::cout << "=== Testing IntelligentMovement2 Ball Interception ===\n" << std::endl;
    
    // Create IntelligentMovement2 instance
    vis::IntelligentMovement2 intelligent_movement2;
    intelligent_movement2.SetPlayerName("robot0");
    intelligent_movement2.SetOpponentName("robot1");
    intelligent_movement2.SetBallName("ball");
    intelligent_movement2.SetAutoMode(true);
    
    // Create mock game objects with proper sizes (100x100 robots)
    std::map<std::string, vis::GameObject> game_objects;
    
    // Create a proper dummy texture
    vis::Texture2D dummy_texture(false); // Initialize with false to avoid loading
    dummy_texture.ID = 1; // Non-zero ID
    
    // Create robot0 (source player) - positioned to intercept
    game_objects["robot0"] = vis::GameObject("robot0", 
                                            glm::vec2(50, 180),    // Position
                                            glm::vec2(100, 100),   // Size (100x100 for emoji)
                                            glm::vec2(0, 0),       // Velocity  
                                            glm::vec2(0, 0),       // Acceleration
                                            1,                     // Mass
                                            dummy_texture);
    
    // Create robot1 (opponent) - positioned between robot0 and ball
    game_objects["robot1"] = vis::GameObject("robot1", 
                                            glm::vec2(25, 100),    // Opponent position
                                            glm::vec2(100, 100),   // Size (100x100 for emoji)
                                            glm::vec2(0, 0),       // Velocity
                                            glm::vec2(0, 0),       // Acceleration
                                            1,                     // Mass
                                            dummy_texture);
    
    // Create moving ball - moving towards robot0
    game_objects["ball"] = vis::GameObject("ball", 
                                          glm::vec2(-50, 0),     // Ball position
                                          glm::vec2(20, 20),     // Size
                                          glm::vec2(80, 60),     // Moving towards robots
                                          glm::vec2(-20, 20),    // Some acceleration
                                          1,                     // Mass
                                          dummy_texture);
    
    std::cout << "Created test scenario:" << std::endl;
    std::cout << "  robot0 (source) at (50, 180)" << std::endl;
    std::cout << "  robot1 (opponent) at (25, 100)" << std::endl;
    std::cout << "  ball at (-50, 0) moving (80, 60)" << std::endl;
    
    // Distance calculations
    float distance_r0_ball = sqrt((50 - (-50))*(50 - (-50)) + (180 - 0)*(180 - 0));
    float distance_r1_ball = sqrt((25 - (-50))*(25 - (-50)) + (100 - 0)*(100 - 0));
    
    std::cout << "  Distance robot0 to ball: " << distance_r0_ball << " pixels" << std::endl;
    std::cout << "  Distance robot1 to ball: " << distance_r1_ball << " pixels" << std::endl;
    std::cout << "  Robot collision radius: 75 pixels (50 + 25 safety)" << std::endl;
    
    std::cout << "\n=== Running Ball Interception Analysis ===\n" << std::endl;
    
    // Simulate a few update cycles
    float dt = 0.1f;
    for (int cycle = 0; cycle < 3; ++cycle) {
        std::cout << "--- Cycle " << (cycle + 1) << " ---" << std::endl;
        
        // Update ball position (simulate movement)
        auto& ball = game_objects["ball"];
        glm::vec2 ball_pos = ball.GetCenterPosition();
        ball_pos += ball.velocity * dt;
        ball.position = ball_pos - ball.size * 0.5f; // Convert center to corner
        
        // Update ball velocity with acceleration
        ball.velocity += ball.acceleration * dt;
        
        std::cout << "Ball now at: (" << ball_pos.x << ", " << ball_pos.y 
                  << ") vel: (" << ball.velocity.x << ", " << ball.velocity.y << ")" << std::endl;
        
        // Run intelligent movement update
        intelligent_movement2.UpdateMovement(game_objects, dt);
        
        std::cout << std::endl;
    }
    
    std::cout << "=== Test Completed ===" << std::endl;
    
    return 0;
}