#ifndef KICK_EXECUTOR_H
#define KICK_EXECUTOR_H

#include "ActionExecutor.h"
#include <Eigen/Dense>

namespace kin {

// Validator for kick actions
class KickValidator {
public:
    bool ValidateKick(const ActionContext& context) const;
    bool ValidatePass(const ActionContext& context, const Eigen::Vector2d& target) const;
};

// Engine for executing kick physics
class KickEngine {
public:
    bool ExecuteKick(const ActionContext& context) const;
    bool ExecutePass(const ActionContext& context, const Eigen::Vector2d& target) const;
};

// Complete kick executor following SRP
class KickExecutor : public ActionExecutor {
private:
    KickValidator validator;
    KickEngine engine;
    
public:
    // Execute kick action
    bool Execute(const ActionContext& context) override;
    
    // Execute pass action (specific method)
    bool ExecutePass(const ActionContext& context, const Eigen::Vector2d& target_position);
    
    // Static convenience methods for backward compatibility
    static bool ExecuteKickAction(std::vector<state::SoccerObject>& soccer_objects);
    static bool ExecutePassAction(std::vector<state::SoccerObject>& soccer_objects, 
                                 const Eigen::Vector2d& target_position);
};

}  // namespace kin

#endif  // KICK_EXECUTOR_H