#include "RobotManager.h"

void rob::RobotManager::KickBall() { robot_action = RobotAction::KICK_BALL; }

void rob::RobotManager::PassBall() { robot_action = RobotAction::PASS_BALL; }