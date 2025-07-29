#include <iostream>

#include "AutoRef.h"
#include "Kinematics.h"
#include "SoccerField.h"
#include "Game.h"
#include "Dimensions.h"

namespace ref {
state::SoccerObject* kicker = nullptr;
bool kicker_released_ball = false;

}  // namespace ref

void ref::CheckCollisions(std::vector<state::SoccerObject>& soccer_objects) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball" && obj.is_attached) {
      state::SoccerObject& robot = *obj.attached_to;
      kin::UpdateAttachedBallPosition(robot, obj);
      kicker = &robot;
      break;
    } else {
      kicker_released_ball = true;
    }
  }

  for (int i = 0; i < soccer_objects.size(); ++i) {
    for (int j = i + 1; j < soccer_objects.size(); ++j) {
      state::SoccerObject& obj1 = soccer_objects[i];
      state::SoccerObject& obj2 = soccer_objects[j];

      if (kin::CheckCircularCollision(obj1, obj2)) {
        if (!(obj1.name == "ball" || obj2.name == "ball")) {
          std::cout << "[ref::CheckCollisions] Detected collision between: " << obj1.name
                    << " and " << obj2.name << std::endl;
        }

        if (obj1.name == "ball") {
          if (kin::IsBallInFrontOfRobot(obj2, obj1)) {
            if (obj1.is_attached) continue;  // Skip only this pair

            kin::HandleBallSticking(obj2, obj1);
            continue;  // Continue to next pair
          }
        }

        if (obj2.name == "ball") {
          if (kin::IsBallInFrontOfRobot(obj1, obj2)) {
            if (obj2.is_attached) continue;  // Skip only this pair

            kin::HandleBallSticking(obj1, obj2);
            continue;  // Continue to next pair
          }
        }
      }
    }
  }

  // ref::CheckForGoals(soccer_objects);

  for (auto& obj : soccer_objects) {
    if (!kin::IsInsideBoundary(obj)) {
      std::cout << "[ref::CheckCollisions] Robot " << obj.name << " is outside the boundary"
                << std::endl;
    }
    if (ref::CheckCollisionWithWall(obj)) {
      std::cout << "[ref::CheckCollisions] Robot " << obj.name << " is colliding with the wall"
                << std::endl;
    }
    if (ref::IsOutsidePlayingField(obj) && !obj.is_attached) {
      std::cout << "[ref::CheckCollisions] Robot " << obj.name << " is outside the playing field"
                << std::endl;
      if (obj.name == "ball") {
        obj.velocity = Eigen::Vector3d::Zero();
        obj.position = Eigen::Vector3d::Zero();
      }
    }
  }
}

bool ref::CheckCollisionWithWall(state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;

  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1] + obj.size[1];
  float bottom = obj.position[1];

  return (left <= -half_width || right >= half_width || top >= half_height ||
          bottom <= -half_height);
}

bool ref::IsOutsidePlayingField(state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;

  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1] + obj.size[1];
  float bottom = obj.position[1];

  return (left < -half_width || right > half_width || top > half_height || bottom < -half_height);
}

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

// bool ref::AttackerDoubleTouchedBall(std::vector<state::SoccerObject>& soccer_objects, Game g) {
//   if (state == Kickoff) {
//     double ball_displacement = 0;
//     for (auto& obj : soccer_objects) {
//       if (obj.name == "ball" && !obj.is_attached) {
//         double x2 = soccer_objects[cfg::SystemConfig::num_robots - 1].position[0];
//         double y2 = soccer_objects[cfg::SystemConfig::num_robots - 1].position[0];
//         ball_displacement = std::sqrt((x2 * x2) + (y2 * y2));
//         std::cout << "[ref::AttackerDoubleTouchedBall] ball displacement: " << ball_displacement
//                   << std::endl;
//       }
//     }
//     if (ball_displacement >= 0.05) {  // 0.05
//       state = Run;
//     } else {
//       if (kicker != nullptr && kicker->is_attached && kicker_released_ball) {
//         std::cout << "[AutoRef::AttackerDoubleTouchedBall] foul" << std::endl;
//         return true;
//       }
//     }

//     return false;
//   }
// }
