#ifndef DRIBBLE_H
#define DRIBBLE_H

#include <Eigen/Dense>
#include "SoccerObject.h"
#include "DribbleExecutor.h"

namespace kin {

bool Dribble(state::SoccerObject& robot, state::Ball& ball, double dribble_power, bool continuous);

// Backward compatibility function - delegate to new architecture
inline bool ExecuteDribble(std::vector<state::SoccerObject>& soccer_objects) {
    return DribbleExecutor::ExecuteDribbleAction(soccer_objects);
}

}  // namespace kin

#endif  // DRIBBLE_H