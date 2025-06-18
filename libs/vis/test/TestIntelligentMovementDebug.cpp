#include "IntelligentMovement.h"
#include "GameObject.h"
#include "ResourceManager.h"
#include <iostream>
#include <map>

int main() {
    std::cout << "=== Testing IntelligentMovement Obstacle Detection ===" << std::endl;
    
    // Create IntelligentMovement instance
    vis::IntelligentMovement intelligent_movement;
    intelligent_movement.SetPlayerName("robot0");
    intelligent_movement.SetBallName("ball");
    intelligent_movement.SetAutoMode(true);
    
    // Create mock game objects similar to real demo
    std::map<std::string, vis::GameObject> game_objects;
    
    // Load a dummy texture (we'll create a simple one)
    vis::Texture2D dummy_texture;
    dummy_texture.ID = 0; // Dummy ID
    
    // Create robot0 (controlled player)
    game_objects["robot0"] = vis::GameObject("robot0", 
                                            glm::vec2(0, 130),     // Position from demo
                                            glm::vec2(25, 25),     // Size
                                            glm::vec2(0, 0),       // Velocity  
                                            glm::vec2(0, 0),       // Acceleration
                                            1,                     // Mass
                                            dummy_texture);
    
    // Create robot1 (should be obstacle)
    game_objects["robot1"] = vis::GameObject("robot1", 
                                            glm::vec2(0, -220),    // Position from demo (130 + -350)
                                            glm::vec2(25, 25),     // Size
                                            glm::vec2(0, 0),       // Velocity
                                            glm::vec2(0, 0),       // Acceleration
                                            1,                     // Mass
                                            dummy_texture);
    
    // Create ball
    game_objects["ball"] = vis::GameObject("ball", 
                                          glm::vec2(-52.5, 0),    // Initial ball position
                                          glm::vec2(10, 10),      // Size
                                          glm::vec2(133, -133),   // Velocity
                                          glm::vec2(-466, 466),   // Acceleration
                                          1,                      // Mass
                                          dummy_texture);
    
    // Create background
    game_objects["background"] = vis::GameObject("background", 
                                                glm::vec2(-400, -300), 
                                                glm::vec2(800, 600),
                                                glm::vec2(0, 0),
                                                glm::vec2(0, 0),
                                                1,
                                                dummy_texture);
    
    std::cout << "\nCreated mock game objects:" << std::endl;
    std::cout << "  robot0 at (0, 130)" << std::endl;
    std::cout << "  robot1 at (0, -220)" << std::endl;
    std::cout << "  ball at (-52.5, 0)" << std::endl;
    std::cout << "  Distance between robots: " << sqrt(350*350) << " units" << std::endl;
    
    std::cout << "\n=== Testing UpdateMovement ===" << std::endl;
    
    // Call UpdateMovement to see what happens
    float dt = 0.1f;
    intelligent_movement.UpdateMovement(game_objects, dt);
    
    std::cout << "\n=== Test Completed ===" << std::endl;
    
    return 0;
}