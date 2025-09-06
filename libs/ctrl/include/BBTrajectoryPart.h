#pragma once

namespace ctrl {

/**
 * @brief Part of a bang-bang trajectory
 * 
 * Direct C++ port of BBTrajectoryPart.java from TIGERs Mannheim
 */
struct BBTrajectoryPart {
    float tEnd{0.0f};  ///< End time of this trajectory part [s]
    float acc{0.0f};   ///< Acceleration during this part [m/s²]
    float v0{0.0f};    ///< Initial velocity of this part [m/s]
    float s0{0.0f};    ///< Initial position of this part [m]
};

} // namespace ctrl