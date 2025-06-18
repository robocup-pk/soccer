#ifndef BALL_INTERCEPT_H
#define BALL_INTERCEPT_H

#include <vector>
#include <memory>
#include "Vector2D.h"

namespace algos {

struct SimplePlayer {
    double x, y, angle;
    
    SimplePlayer(double x = 0.0, double y = 0.0, double angle = 0.0) 
        : x(x), y(y), angle(angle) {}
    
    Vector2D position() const { return Vector2D(x, y); }
};

struct SimpleBall {
    double x, y, vx, vy;
    
    SimpleBall(double x = 0.0, double y = 0.0, double vx = 0.0, double vy = 0.0) 
        : x(x), y(y), vx(vx), vy(vy) {}
    
    Vector2D position() const { return Vector2D(x, y); }
    Vector2D velocity() const { return Vector2D(vx, vy); }
    Vector2D predictPosition(int steps) const;
};

struct InterceptCandidate {
    Vector2D target_position;
    int steps_to_reach;
    double evaluation_score;
    bool requires_turn;
    double turn_angle;
    
    InterceptCandidate() : steps_to_reach(0), evaluation_score(0.0), requires_turn(false), turn_angle(0.0) {}
};

class InterceptEvaluator {
public:
    struct EvaluationParams {
        double shoot_spot_weight;
        double opponent_step_weight;
        double teammate_step_weight;
        double turn_penalty_weight;
        double distance_weight;
        double ball_speed_weight;
        
        EvaluationParams() : shoot_spot_weight(1.0), opponent_step_weight(5.0), teammate_step_weight(2.0),
                           turn_penalty_weight(1.5), distance_weight(1.0), ball_speed_weight(0.8) {}
    };

private:
    EvaluationParams params_;

public:
    InterceptEvaluator(const EvaluationParams& params = EvaluationParams()) : params_(params) {}
    
    double evaluateCandidate(const InterceptCandidate& candidate, 
                           const SimplePlayer& player,
                           const SimpleBall& ball,
                           const std::vector<SimplePlayer>& opponents,
                           int opponent_min_step = 1000,
                           int teammate_min_step = 1000) const;
    
    double calculateShootSpotValue(const Vector2D& position) const;
    double calculateOpponentStepPenalty(int self_step, int opponent_step) const;
    double calculateTeammateStepPenalty(int self_step, int teammate_step) const;
    double calculateTurnPenalty(double turn_angle) const;
    double calculateDistancePenalty(double distance) const;
    double calculateBallSpeedBonus(const SimpleBall& ball, int steps) const;

private:
};

class BallInterceptor {
public:
    struct InterceptParams {
        int max_predict_steps;
        double collision_radius;
        int max_candidates;
        bool enable_obstacle_avoidance;
        double player_max_speed;
        
        InterceptParams() : max_predict_steps(50), collision_radius(0.5), 
                          max_candidates(20), enable_obstacle_avoidance(true), player_max_speed(1.0) {}
    };

private:
    InterceptParams params_;
    std::unique_ptr<InterceptEvaluator> evaluator_;

public:
    BallInterceptor(const InterceptParams& params = InterceptParams());
    ~BallInterceptor();
    
    std::vector<InterceptCandidate> generateCandidates(const SimplePlayer& player, 
                                                     const SimpleBall& ball,
                                                     const std::vector<SimplePlayer>& opponents) const;
    
    InterceptCandidate getBestIntercept(const SimplePlayer& player, 
                                      const SimpleBall& ball,
                                      const std::vector<SimplePlayer>& opponents,
                                      int opponent_min_step = 1000,
                                      int teammate_min_step = 1000) const;
    
    bool canIntercept(const SimplePlayer& player, const SimpleBall& ball, int max_steps = 10) const;
    
    Vector2D calculateAvoidanceForce(const SimplePlayer& player, 
                                   const std::vector<SimplePlayer>& opponents,
                                   const Vector2D& desired_direction) const;
    
    int calculateStepsToReach(const SimplePlayer& player, const Vector2D& target, 
                            const std::vector<SimplePlayer>& opponents) const;
    
    bool isPathClear(const Vector2D& start, const Vector2D& end, 
                    const std::vector<SimplePlayer>& opponents, int step_offset = 0) const;
    
    Vector2D findOptimalPath(const SimplePlayer& player, const Vector2D& target,
                           const std::vector<SimplePlayer>& opponents) const;
    
    double calculateRequiredTurn(const SimplePlayer& player, const Vector2D& target) const;

private:
};

class BallInterceptAction {
public:
    enum ActionType {
        DASH,
        TURN,
        TURN_AND_DASH,
        WAIT
    };
    
    struct Action {
        ActionType type;
        double power;
        double angle;
        Vector2D direction;
        Vector2D target_position;
        
        Action(ActionType t = WAIT, double p = 0.0, double a = 0.0, 
               const Vector2D& dir = Vector2D(), const Vector2D& target = Vector2D())
            : type(t), power(p), angle(a), direction(dir), target_position(target) {}
    };

private:
    std::unique_ptr<BallInterceptor> interceptor_;

public:
    BallInterceptAction();
    BallInterceptAction(const BallInterceptor::InterceptParams& params);
    ~BallInterceptAction();
    
    Action execute(const SimplePlayer& player, 
                  const SimpleBall& ball,
                  const std::vector<SimplePlayer>& opponents,
                  const std::vector<SimplePlayer>& teammates = {},
                  int opponent_min_step = 1000,
                  int teammate_min_step = 1000);
    
    bool shouldChase(const SimplePlayer& player, 
                    const SimpleBall& ball,
                    int self_min_step,
                    int teammate_min_step, 
                    int opponent_min_step,
                    int pressing_factor = 10) const;
    
    int calculateMinInterceptSteps(const SimplePlayer& player, const SimpleBall& ball) const;

private:
    Action createDashAction(const SimplePlayer& player, const Vector2D& target, 
                          const std::vector<SimplePlayer>& opponents);
    
    Action createTurnAction(const SimplePlayer& player, const Vector2D& target);
    
    double calculateOptimalDashPower(const SimplePlayer& player, const Vector2D& target, 
                                   double distance, int steps) const;
};

}

#endif // BALL_INTERCEPT_H