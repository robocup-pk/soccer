#ifndef CALCVEL_H
#define CALCVEL_H
#include <vector>
#include <string>

namespace vel {

class VelocityControl {
 public:
  VelocityControl(float time, float destination, float max_acc = 1, float max_deceleration = 2.5,
                  float origin = 0, float Top_speed = 3);
  int clip(int Time) const;
  void CalVel();
  float fetch(int T) const;
  float fetch_exact(float time, std::string type = "Constant") const;
  float check(float T) const;

 private:
  float origin;
  float max_acc;
  float max_deceleration;
  float time;
  float destination;
  float Top_speed;

  std::vector<float> final_answer;
};

}  // namespace vel

#endif