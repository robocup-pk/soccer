#include "Game.h"
#include "SoccerField.h"
#include <iostream>
#include "GLSimulation.h"

#include "Kinematics.h"
#include "Dimensions.h"

ref::Game::Game() { state = Kickoff; }

void ref::Game::MoveToFormation(std::vector<Eigen::Vector3d> team_one_formation,
                                std::vector<Eigen::Vector3d> team_two_formation,
                                std::vector<state::SoccerObject>& soccer_objects) {
  if (soccer_objects.size() > 12) {
    for (int i = 0; i < soccer_objects.size() / 2; i++) {
      soccer_objects[i].position = cfg::SystemConfig::team_one_start_formation[i];
      soccer_objects[i + soccer_objects.size() / 2].position =
          cfg::SystemConfig::team_two_start_formation[i];
    }
  }
}

void ref::Game::CheckCollisions(std::vector<state::SoccerObject>& soccer_objects) {
  for (auto& obj : soccer_objects) {
    if (obj.name == "ball" && obj.is_attached) {
      state::SoccerObject& robot = *obj.attached_to;
      kin::UpdateAttachedBallPosition(robot, obj);
      break;
    }
  }

  for (auto& obj : soccer_objects) {
    if (obj.name == "ball" && !obj.is_attached) {
      kicker_released = true;
      break;
    }
  }

  for (int i = 0; i < soccer_objects.size(); ++i) {
    for (int j = i + 1; j < soccer_objects.size(); ++j) {
      state::SoccerObject& obj1 = soccer_objects[i];
      state::SoccerObject& obj2 = soccer_objects[j];

      if (kin::CheckCircularCollision(obj1, obj2)) {
        if (!(obj1.name == "ball" || obj2.name == "ball")) {
          // std::cout << "[ref::CheckCollisions] Detected collision between: " << obj1.name
          //           << " and " << obj2.name << std::endl;
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
    if (ref::Game::CheckCollisionWithWall(obj)) {
      std::cout << "[ref::CheckCollisions] Robot " << obj.name << " is colliding with the wall"
                << std::endl;
    }
    if (ref::Game::IsOutsidePlayingField(obj) && !obj.is_attached) {
      std::cout << "[ref::CheckCollisions] Robot " << obj.name << " is outside the playing field"
                << std::endl;
      if (obj.name == "ball") {
        obj.velocity = Eigen::Vector3d::Zero();
        obj.position = Eigen::Vector3d::Zero();
      }
    }
  }
}

void ref::Game::DoGoals(std::vector<state::SoccerObject>& soccer_objects) {
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
        goal_scored = true;
        team_with_ball = 1;
        std::cout << "[ref::Game::DoGoals] RED TEAM SCORED! "
                  << "red team score: " << team_two_score << std::endl;
      } else if (right >= right_goal_x1 && right <= right_goal_x2 && top <= right_goal_y1 &&
                 bottom >= right_goal_y2) {
        team_one_score++;
        goal_scored = true;
        team_with_ball = 2;
        std::cout << "[ref::Game::DoGoals] YELLOW TEAM SCORED! "
                  << "yellow team score: " << team_one_score << std::endl;
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

        state = Game::Kickoff;
        kicker_released = false;
        std::cout << "[ref::Game::DoGoals] GAME STATE IS KICKOFF" << std::endl;
        state_start_ball_pos = soccer_objects[soccer_objects.size() - 1].position;
        vis::team_one_selected_player = 0;
        vis::team_two_selected_player = cfg::SystemConfig::num_robots / 2;

        if (team_with_ball == 1) {
          cfg::SystemConfig::team_one_start_formation[0] = Eigen::Vector3d(
              -1 * cfg::SystemConfig::ball_radius_m - cfg::SystemConfig::robot_size_m[0] / 2.0,
              0.0f, 0);
          cfg::SystemConfig::team_two_start_formation[0] = Eigen::Vector3d(0.5f, 0.0f, M_PI);
          soccer_objects[soccer_objects.size() - 1].attached_to =
              &soccer_objects[cfg::SystemConfig::team_one_kicker];
          kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_one_kicker],
                                  soccer_objects[soccer_objects.size() - 1]);
        } else if (team_with_ball == 2) {
          cfg::SystemConfig::team_two_start_formation[0] = Eigen::Vector3d(
              cfg::SystemConfig::ball_radius_m + cfg::SystemConfig::robot_size_m[0] / 2.0, 0.0f,
              M_PI);
          cfg::SystemConfig::team_one_start_formation[0] = Eigen::Vector3d(-0.5f, 0.0f, 0);
          soccer_objects[soccer_objects.size() - 1].attached_to =
              &soccer_objects[cfg::SystemConfig::team_two_kicker];
          kin::HandleBallSticking(soccer_objects[cfg::SystemConfig::team_two_kicker],
                                  soccer_objects[soccer_objects.size() - 1]);
        }

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

void ref::Game::UpdateGameState(std::vector<state::SoccerObject>& soccer_objects) {
  if (state == Kickoff) {
    Eigen::Vector3d ball_displacement =
        state_start_ball_pos - soccer_objects[soccer_objects.size() - 1].position;
    if (ball_displacement.norm() > 0.1) {
      state = Run;
      std::cout << "[ref::Game::UpdateGameState] GAME STATE IS RUN" << std::endl;
    }
  }
}

bool ref::Game::CheckCollisionWithWall(state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().width_mm / 2) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().height_mm / 2) / 1000.0f;

  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1] + obj.size[1];
  float bottom = obj.position[1];

  return (left <= -half_width || right >= half_width || top >= half_height ||
          bottom <= -half_height);
}

bool ref::Game::IsOutsidePlayingField(state::SoccerObject& obj) {
  float half_width = (vis::SoccerField::GetInstance().playing_area_width_mm / 2.0f) / 1000.0f;
  float half_height = (vis::SoccerField::GetInstance().playing_area_height_mm / 2.0f) / 1000.0f;

  float left = obj.position[0];
  float right = obj.position[0] + obj.size[0];
  float top = obj.position[1] + obj.size[1];
  float bottom = obj.position[1];

  return (left < -half_width || right > half_width || top > half_height || bottom < -half_height);
}