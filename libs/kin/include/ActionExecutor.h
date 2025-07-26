#ifndef ACTION_EXECUTOR_H
#define ACTION_EXECUTOR_H

#include <Eigen/Dense>
#include "SoccerObject.h"

namespace kin {

// Configuration constants to replace magic numbers
namespace config {
    static constexpr double DEFAULT_ACTION_DISTANCE = 0.3;    // 30cm action range
    static constexpr double KICK_POWER_DEFAULT = 5.0;         // Default kick power
    static constexpr double PASS_POWER_DEFAULT = 3.0;         // Default pass power
    static constexpr double DRIBBLE_POWER_DEFAULT = 15.0;     // Default dribble power
    static constexpr double ANGLE_TOLERANCE_RAD = M_PI/3;     // 60 degrees tolerance
}

// Context for action execution
struct ActionContext {
    state::SoccerObject& robot;
    state::SoccerObject& ball;
    double power = 0.0;
    Eigen::Vector2d target_position = Eigen::Vector2d::Zero();
    bool force_execute = false;
    
    ActionContext(state::SoccerObject& r, state::SoccerObject& b) : robot(r), ball(b) {}
};

// Utility class for common action validation methods
class ActionUtils {
public:
    static bool IsWithinRange(const state::SoccerObject& robot, const state::SoccerObject& ball, 
                             double max_distance = config::DEFAULT_ACTION_DISTANCE);
    
    static bool IsRobotFacingBall(const state::SoccerObject& robot, const state::SoccerObject& ball,
                                 double angle_tolerance = config::ANGLE_TOLERANCE_RAD);
    
    static double CalculateDistance(const state::SoccerObject& obj1, const state::SoccerObject& obj2);
};

// Abstract base class for all action executors
class ActionExecutor {
public:
    virtual ~ActionExecutor() = default;
    
    // Execute the action with given context
    virtual bool Execute(const ActionContext& context) = 0;
};

// Helper class to find objects in soccer_objects vector (for backward compatibility)
class ObjectFinder {
public:
    static state::SoccerObject* FindRobot(std::vector<state::SoccerObject>& objects, 
                                          const std::string& robot_name = "robot0");
    
    static state::SoccerObject* FindBall(std::vector<state::SoccerObject>& objects);
    
    // Find both robot and ball, return true if both found
    static bool FindRobotAndBall(std::vector<state::SoccerObject>& objects,
                                state::SoccerObject*& robot, state::SoccerObject*& ball,
                                const std::string& robot_name = "robot0");
};

}  // namespace kin

#endif  // ACTION_EXECUTOR_H