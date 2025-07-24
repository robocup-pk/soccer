#ifndef DRIBBLE_H
#define DRIBBLE_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace kin {

bool Dribble(state::SoccerObject& robot, state::Ball& ball, double dribble_power, bool continuous);

}  // namespace kin

#endif  // DRIBBLE_H