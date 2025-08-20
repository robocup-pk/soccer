#include "Controller.h"

namespace ref {

void Controller::DoFouls(std::vector<state::SoccerObject>& soccer_objects, Game& g,
                         ref::AutoRef& referee) {
  if (g.state != ref::Game::Stop && g.state != ref::Game::Halt) {
    // Other non-foul violations
    // referee.ViolatedKickOffSetUp(soccer_objects, 1, game);
    // referee.ViolatedKickOffSetUp(soccer_objects, 2, game);

    // #1 ATTACKER DOUBLE TOUCHED BALL
    int double_touch_foul = referee.AttackerDoubleTouchedBall(soccer_objects, g);
    if (double_touch_foul == 1 || double_touch_foul == 2) {
      g.SetUpFreeKick(soccer_objects, soccer_objects[soccer_objects.size() - 1].position);
    }

    // #3 BALL LEFT FIELD TOUCH LINE + #7 BOUNDARY CROSSING
    if (referee.BallLeftFieldTouchLine(soccer_objects, g)) {
      Eigen::Vector3d new_pos = soccer_objects[soccer_objects.size() - 1].position;

      if (new_pos[1] >= 0) {  // top touch line crossed
        new_pos[1] = vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 - 0.2;
      } else {
        new_pos[1] = -1 * vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 + 0.2;
      }
      new_pos[0] = std::clamp(
          new_pos[0], -1 * vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 + 0.2,
          vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 - 0.2);

      g.SetUpFreeKick(soccer_objects, new_pos);
    }

    // #3 BALL LEFT FIELD GOAL LINE +  #5 AIMLESS KICK + #7 BOUNDARY CROSSING
    std::vector<bool> b = referee.BallLeftFieldGoalLines(soccer_objects, g);

    // #2 CHECK GOALS
    if ((b[0] != true && b[1] != true && b[2] != true) &&
        b[3]) {  // crossed from left not aimless so scoring team is two
      std::cout << "[ref::AutoRef::CheckGoals] *RED* scored. score: " << g.team_two_score + 1
                << std::endl;
      g.DoGoals(soccer_objects, 2);
    } else if ((b[0] != true && b[1] != true && b[2] != true) &&
               b[4]) {  // crossed from right not aimless so scoring team is one
      std::cout << "[ref::AutoRef::CheckGoals] *YELLOW* scored. score: " << g.team_one_score + 1
                << std::endl;
      g.DoGoals(soccer_objects, 1);
    } else if (b[0] == true || b[1] == true) {  // its an aimless kick
      g.SetUpFreeKick(soccer_objects, vis::last_ball_kick_pos);
    } else if (b[2] == true) {  // its a boundary crossing
      if ((g.last_bot_touched_ball.team_id == 2 && b[3]) ||
          (g.last_bot_touched_ball.team_id == 1 && b[4])) {  // goal kick awarded
        // calculating goal kick pos
        Eigen::Vector3d new_pos = soccer_objects[soccer_objects.size() - 1].position;
        if (new_pos[1] >= 0) {  // top touch line crossed
          new_pos[1] = vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 - 0.2;
        } else {
          new_pos[1] = -1 * vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 + 0.2;
        }
        new_pos[0] = std::clamp(
            new_pos[0], -1 * vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 + 1.0,
            vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 - 1);
        new_pos[1] = std::clamp(
            new_pos[1], -1 * vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 + 0.2,
            vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 - 0.2);
        std::cout << "[Controller::DoFouls] goal kick awarded" << std::endl;
        g.SetUpFreeKick(soccer_objects, new_pos);

      } else if ((g.last_bot_touched_ball.team_id == 2 && b[4]) ||
                 (g.last_bot_touched_ball.team_id == 1 && b[3])) {  // corner kick awarded
        // figure out the position of the freekick
        if ((g.last_bot_touched_ball.team_id == 2 && b[3]) ||
            (g.last_bot_touched_ball.team_id == 1 && b[4])) {  // goal kick awarded
          Eigen::Vector3d new_pos = soccer_objects[soccer_objects.size() - 1].position;
          if (new_pos[1] >= 0) {  // top touch line crossed
            new_pos[1] = vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 - 0.2;
          } else {
            new_pos[1] =
                -1 * vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 + 0.2;
          }
          new_pos[0] =
              std::clamp(new_pos[0],
                         -1 * vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 + 0.2,
                         vis::SoccerField::GetInstance().playing_area_width_mm / 2000.0 - 0.2);
          new_pos[1] = std::clamp(
              new_pos[1],
              -1 * vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 + 0.2,
              vis::SoccerField::GetInstance().playing_area_height_mm / 2000.0 - 0.2);

          std::cout << "[Controller::DoFouls] corner kick awarded" << std::endl;
          g.SetUpFreeKick(soccer_objects, new_pos);
        }
      }
    }
    // #6 DEFENDER IN DEFENSE AREA
    // #8 BOT DRIBBLED BALL TOO FAR
    if (referee.BotDribbledBallTooFar(soccer_objects, g)) {
      g.SetUpFreeKick(soccer_objects, g.last_valid_pos);
    }
  }

  referee.BotTooFastInStop(soccer_objects, g);
}
}  // namespace ref