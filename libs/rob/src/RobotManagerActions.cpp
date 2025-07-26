#define _USE_MATH_DEFINES
#include "RobotManager.h"
#include <cmath>
#include <iostream>

void rob::RobotManager::KickBall() { 
    robot_action = RobotAction::KICK_BALL; 
}

void rob::RobotManager::PassBall() { 
    robot_action = RobotAction::PASS_BALL; 
}

