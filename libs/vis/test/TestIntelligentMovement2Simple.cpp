#include "IntelligentMovement2.h"
#include <iostream>

int main() {
    std::cout << "=== Simple IntelligentMovement2 Test ===\n" << std::endl;
    
    try {
        // Just test the constructor to see if ball_intercept works
        std::cout << "Creating IntelligentMovement2..." << std::endl;
        vis::IntelligentMovement2 intelligent_movement2;
        
        std::cout << "Setting configuration..." << std::endl;
        intelligent_movement2.SetPlayerName("robot0");
        intelligent_movement2.SetOpponentName("robot1");
        intelligent_movement2.SetBallName("ball");
        
        std::cout << "IntelligentMovement2 created successfully!" << std::endl;
        std::cout << "Auto mode enabled: " << intelligent_movement2.IsAutoModeEnabled() << std::endl;
        
        std::cout << "=== Test Completed Successfully ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }
    
    return 0;
}