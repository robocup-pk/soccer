#include <iostream>

#include "GLConfig.h"
#include "Collision.h"

void vis::CheckCollision(std::vector<GameObject>& game_objects) {
  // We can have collisions between: robot/robot and robot/ball

  // Let's just do collision between ball and wall:
  for (GameObject& g : game_objects) {
    if (g.name == "ball") {
      int ball_x = g.GetCenterPosition().x;
      int ball_y = g.GetCenterPosition().y;

      std::cout << "Ball Position: " << ball_x << " " << ball_y << std::endl;

      // x direction
      if (ball_x + g.size.x / 2 >= GLConfig::window_width_px / 2) {
        if (g.velocity.x > 0) {
          g.velocity.x *= -1;
        }
      }
      if (ball_x - g.size.x / 2 <= -GLConfig::window_width_px / 2) {
        std::cout << "Collision with left wall" << std::endl;
        if (g.velocity.x < 0) {
          g.velocity.x *= -1;
        }
      }

      // y direction
      if (ball_y + g.size.y / 2 >= GLConfig::window_height_px / 2) {
        if (g.velocity.y > 0) {
          g.velocity.y *= -1;
        }
      }
      if (ball_y - g.size.y / 2 <= -GLConfig::window_height_px / 2) {
        if (g.velocity.y < 0) {
          g.velocity.y *= -1;
        }
      }

      break;
    } else if (g.name == "robot0") {
      // std::cout << "robot0: " << g.GetCenterPosition().x << " " << g.GetCenterPosition().y
      //           << std::endl;
    }
  }

  // for (int i = 0; i < game_objects.size() - 1; ++i) {
  //   if (game_objects[i].name == "background") continue;
  //   for (int j = 1; j < game_objects.size(); ++j) {
  //     if (game_objects[i].name == "background") continue;

  //     // Positions of both the game objects
  //     glm::vec2 first_object_position = game_objects[i].GetCenterPosition();
  //     glm::vec2 second_object_position = game_objects[i].GetCenterPosition();

  //     // Check if they collide
  //   }
  // }
}