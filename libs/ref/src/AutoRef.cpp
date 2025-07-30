#include <iostream>

#include "AutoRef.h"
#include "Kinematics.h"
#include "SoccerField.h"
#include "Game.h"
#include "Dimensions.h"

// GAME EVENTS

// Violated KickOff Configuration
/*
  When the kick-off command is issued, all robots have to move to their own half of the field
  excluding the center circle. However, one robot of the attacking team is also allowed to be
  inside the whole center circle. This robot will be referred to as the kicker. No robot is allowed
  to touch the ball.
*/

bool ref::ViolatedKickOffSetUp(std::vector<state::SoccerObject>& soccer_objects, int team_id,
                               Game& g) {
  if (g.state != ref::Game::Kickoff) {
    return false;
  }

  if (team_id == 1) {
    for (int i = 0; i < soccer_objects.size(); i++) {
      if (soccer_objects[i].team_id == 1 &&
          (soccer_objects[i].position[0] + cfg::SystemConfig::robot_size_m[0] / 2.0 > 0 ||
           (g.team_with_ball == 1 && RobotInCenterCircle(soccer_objects[i].position) &&
            i != cfg::SystemConfig::team_one_kicker) ||
           (g.team_with_ball != 1 && RobotInCenterCircle(soccer_objects[i].position)))) {
        std::cout << "[ref::ViolatedKickOffSetUp] team 1 violated kickoff start arrangement"
                  << std::endl;
        return true;
      }
    }
  } else {
    for (int i = 0; i < soccer_objects.size(); i++) {
      if (soccer_objects[i].team_id == 2 &&
          (soccer_objects[i].position[0] - cfg::SystemConfig::robot_size_m[0] / 2.0 < 0 ||
           (g.team_with_ball == 2 && RobotInCenterCircle(soccer_objects[i].position) &&
            i != cfg::SystemConfig::team_two_kicker) ||
           (g.team_with_ball != 2 && RobotInCenterCircle(soccer_objects[i].position)))) {
        std::cout << "[ref::ViolatedKickOffSetUp] team 2 violated kickoff start arrangement"
                  << std::endl;
        return true;
      }
    }
  }

  return false;
}

/* helper for ViolateKickOffSetUp */
bool ref::RobotInCenterCircle(Eigen::Vector3d v) {
  double x = v[0];
  double y = v[1];
  double r_1 = cfg::SystemConfig::robot_size_m[0] / 2.0;
  double r_2 = 0.2;

  if (std::sqrt(std::abs(x * x) + std::abs(y * y)) <= r_1 + r_2) {
    return true;
  }
  return false;
}

// #1 ATTACKER DOUBLE TOUCHED BALL

/* When the ball is brought into play following a kick-off or free kick, the kicker is not
allowed to touch the ball until it has been touched by another robot or the game has been
stopped. The ball must have moved at least 0.05 meters to be considered as in play. A double
touch results in a stop followed by a free kick from the same ball position. */

bool ref::AttackerDoubleTouchedBall(std::vector<state::SoccerObject>& soccer_objects, Game& g) {
  if (g.state == ref::Game::Kickoff) {
    if (g.team_with_ball == 1) {
      if (soccer_objects[cfg::SystemConfig::team_one_kicker].is_attached && g.kicker_released) {
        std::cout << "[AutoRef::AttackerDoubleTouchedBall] Team 1 Double touch foul" << std::endl;
        return true;
      }
    } else if (g.team_with_ball == 2) {
      if (soccer_objects[cfg::SystemConfig::team_two_kicker].is_attached && g.kicker_released) {
        std::cout << "[AutoRef::AttackerDoubleTouchedBall] Team 2 Double touch foul" << std::endl;
        return true;
      }
    }
  }
  return false;
}
