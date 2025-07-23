#include "Dribble.h"
#include "Kinematics.h"
#include "BallModel.h"
#include <cmath>
#include <iostream>

namespace kin {

bool Dribble(state::SoccerObject& robot, state::SoccerObject& ball,
             double dribble_power, bool force_dribble) {
    Eigen::Vector3d robot_center = robot.GetCenterPosition();
    Eigen::Vector3d ball_center = ball.GetCenterPosition();
    double distance = std::hypot(ball_center.x() - robot_center.x(),
                                 ball_center.y() - robot_center.y());

    if (distance > ssl::MAX_DRIBBLE_DISTANCE && !force_dribble)
        return false;

    Eigen::Vector2d ball_point(ball_center.x(), ball_center.y());
    if (!robot.IsPointInFrontSector(ball_point) && !force_dribble)
        return false;

    if (ball.is_attached)
        DetachBall(ball, 0.0f);

    Eigen::Vector2d dir(std::cos(robot.position[2]), std::sin(robot.position[2]));
    dir.normalize();
    global_ball_model.ApplyDribbleForce(ball.velocity, dir * dribble_power);
    return true;
}

} // namespace kin
