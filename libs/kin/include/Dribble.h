#ifndef DRIBBLE_H
#define DRIBBLE_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace kin {

namespace ssl {
static constexpr double MAX_DRIBBLE_DISTANCE = 0.3; // 30 cm typical dribble range
}

// Apply a dribble force to the ball without attaching it to the robot.
// Returns true if the dribble was applied (ball in range and in front) or force_dribble is true.
bool Dribble(state::SoccerObject& robot, state::SoccerObject& ball,
             double dribble_power = 1.0, bool force_dribble = false);

} // namespace kin

#endif // DRIBBLE_H
