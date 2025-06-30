#include "CalcVel.h"
#include <algorithm>
#include <iostream>
namespace vel {

VelocityControl::VelocityControl(float time, float destination, float max_acc,
                                 float max_deceleration, float origin, float Top_speed)
    : origin(origin),
      max_acc(max_acc),
      time(time),
      destination(destination),
      max_deceleration(max_deceleration),
      Top_speed(Top_speed) {
  CalVel();
}

float VelocityControl::check(float T) const {
  float C2 = T * max_deceleration;
  float C1 = 0;
  float Common_x = (C2) / (max_acc - max_deceleration);
  float Common_y = max_acc * Common_x + C1;
  if (Common_y <= Top_speed) {
    float Area_under_graph = (0.5 * Common_x * Common_y) + (0.5 * (T - Common_x) * Common_y);
    if (destination - 0.5 <= Area_under_graph && Area_under_graph <= destination + 0.5) {
      return 1;
    }
  } else if (Common_y > Top_speed) {
    float x1 = (Top_speed - C1) / max_acc;
    float x2 = (Top_speed - C2) / max_deceleration;
    float Area_under_graph =
        (0.5 * x1 * Top_speed) + (0.5 * (T - x2) * Top_speed) + (Top_speed * (x2 - x1));
    if (destination - 0.5 <= Area_under_graph && Area_under_graph <= destination + 0.5) {
      return 1;
    }
  }

  return 0.0;
}
int VelocityControl::clip(int T) const { return std::clamp(T, 0, static_cast<int>(time)); };

void VelocityControl::CalVel() {
  if (!check(time)) {
    std::cout << "ERROR, The robot is too slow to reach the destination in the given time."
              << std::endl;
    return;
  }
  sdd
  

  return;
};

float VelocityControl::fetch(int T) const {
  T = clip(T);
  return final_answer[T];
};

float VelocityControl::fetch_exact(float time, std::string type) const { return 0.0; };

}  // namespace vel
