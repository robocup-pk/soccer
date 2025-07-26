#ifndef DRIBBLE_EXECUTOR_H
#define DRIBBLE_EXECUTOR_H

#include "ActionExecutor.h"
#include "BallModel.h"

namespace kin {

// Validator for dribble actions (SSL compliance)
class DribbleValidator {
public:
    bool ValidateDribble(const ActionContext& context) const;
};

// Engine for executing dribble physics
class DribbleEngine {
public:
    bool ExecuteDribble(const ActionContext& context) const;
    
private:
    void ApplyVelocityMatchingForce(const ActionContext& context) const;
    void ApplyPullForce(const ActionContext& context, double distance) const;
};

// Complete dribble executor following SRP
class DribbleExecutor : public ActionExecutor {
private:
    DribbleValidator validator;
    DribbleEngine engine;
    
public:
    // Execute dribble action
    bool Execute(const ActionContext& context) override;
    
    // Static convenience method for backward compatibility
    static bool ExecuteDribbleAction(std::vector<state::SoccerObject>& soccer_objects);
};

}  // namespace kin

#endif  // DRIBBLE_EXECUTOR_H