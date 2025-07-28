#include <iostream>

#include "AutoRef.h"
#include "Kinematics.h"
#include "SoccerField.h"
#include "Game.h"

namespace ref {
int team_one_score = 0;
int team_two_score = 0;
}  // namespace ref

void ref::CheckCollisions(std::vector<state::SoccerObject>& soccer_objects) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball" && obj.is_attached) {
      state::SoccerObject& robot = *obj.attached_to;
      kin::UpdateAttachedBallPosition(robot, obj);
      break;
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

  ref::CheckForGoals(soccer_objects);

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

void ref::CheckForGoals(std::vector<state::SoccerObject>& soccer_objects) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball") {
      double left_goal_x1 = -(((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) +
                               vis::SoccerField::GetInstance().goal_width_mm)) /
                            1000.0f;
      double left_goal_x2 =
          -((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f)) / 1000.0f;

      double left_goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
      double left_goal_y2 = -((vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f);

      double right_goal_x1 =
          (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
      double right_goal_x2 = ((vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) +
                              vis::SoccerField::GetInstance().goal_width_mm) /
                             1000.0f;

      double right_goal_y1 = (vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f;
      double right_goal_y2 = -((vis::SoccerField::GetInstance().goal_height_mm / 2.0f) / 1000.0f);

      bool goal_scored = false;
      float left = obj.position[0];
      float right = obj.position[0] + obj.size[0];
      float top = obj.position[1] + obj.size[1];
      float bottom = obj.position[1];

      if ((left >= left_goal_x1 && left <= left_goal_x2 && top <= left_goal_y1 &&
           bottom >= left_goal_y2) ||
          (right >= left_goal_x1 && right <= left_goal_x2 && top >= left_goal_y1 &&
           bottom <= left_goal_y2)) {
        team_two_score++;
        std::cout << "[ref::CheckForGoals] Goal by red team. Red team score:  " << team_two_score
                  << std::endl;
        goal_scored = true;
      } else if (right >= right_goal_x1 && right <= right_goal_x2 && top <= right_goal_y1 &&
                 bottom >= right_goal_y2) {
        team_one_score++;
        std::cout << "[ref::CheckForGoals] Goal by yellow team. Yellow team score:  "
                  << team_one_score << std::endl;
        goal_scored = true;
      }

      if (goal_scored) {
        if (obj.is_attached) {
          obj.is_attached = false;
          state::SoccerObject* robot = obj.attached_to;
          if (robot) {
            robot->is_attached = false;
            robot->attached_to = nullptr;
          }
          obj.attached_to = nullptr;
        }

        obj.position = Eigen::Vector3d::Zero();
        obj.velocity = Eigen::Vector3d::Zero();

        std::cout << "[ref::CheckForGoals] Ball position reset to origin after goal." << std::endl;
        ref::Game::MoveToFormation(cfg::SystemConfig::team_one_start_formation,
                                   cfg::SystemConfig::team_two_start_formation, soccer_objects);

        for (auto& obj : soccer_objects) {
          obj.velocity = Eigen::Vector3d(0, 0, 0);
        }
      }

      break;
    }
  }
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

ref::AutoRef::AutoRef() {
  team_one_score = 0;
  team_two_score = 0;
  state = Kickoff;
}

// GAME EVENTS

// ATTACKER DOUBLE TOUCHED BALL

/* When the ball is brought into play following a kick-off or free kick, the kicker is not
allowed to touch the ball until it has been touched by another robot or the game has been
stopped. The ball must have moved at least 0.05 meters to be considered as in play. A double
touch results in a stop followed by a free kick from the same ball position. */

bool ref::AutoRef::AttackerDoubleTouchedBall(GameState state, bool released_ball,
                                             double disp_ball_since_kickoff,
                                             state::SoccerObject& player) {
  if (state == Kickoff && released_ball &&
      disp_ball_since_kickoff <
          0.05) {  // player = player who initiated kickoff, released_ball is if it kicked ball
    if (player.is_attached) {  // is attached is for if it double touches the ball again after it
      // released it
      return true;
    }
  }
  return false;
}
