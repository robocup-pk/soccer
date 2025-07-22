#ifndef KICK_H
#define KICK_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace kin {

// Enhanced SSL RoboCup kick function
// Implements proper SSL constraints: orientation check, distance limits, power regulation
// Returns true if kick was successful, false if constraints not met
bool Kick(state::SoccerObject& robot, state::SoccerObject& ball, 
          double kick_power = 3.0, bool force_kick = false);

// SSL-specific kick parameters and constraints
namespace ssl {
    static constexpr double MAX_KICK_DISTANCE = 0.25;    // 25cm - SSL typical kicking range
    static constexpr double MAX_KICK_POWER = 6.0;        // 6 m/s - SSL maximum kick speed
    static constexpr double DEFAULT_KICK_POWER = 3.0;    // 3 m/s - SSL typical kick speed
    static constexpr double DEFAULT_PASS_POWER = 2.0;    // 2 m/s - SSL typical pass speed
    static constexpr int KICK_DELAY_MS = 50;             // 50ms post-kick delay
}

}  // namespace kin

#endif  // KICK_H