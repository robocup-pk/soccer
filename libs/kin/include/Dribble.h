#ifndef DRIBBLE_H
#define DRIBBLE_H

#include <Eigen/Dense>

namespace state {
class SoccerObject;
}

namespace kin {

// Simple dribble function that applies force to keep ball close to robot
bool Dribble(state::SoccerObject& robot, state::SoccerObject& ball, double power, bool continuous = false);

}  // namespace kin

#endif  // DRIBBLE_H