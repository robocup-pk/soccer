#include "ball_intercept.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

using namespace std;

namespace algos {

Vector2D SimpleBall::predictPosition(int steps) const {
    if (steps <= 0) return Vector2D(x, y);
    
    const double BALL_DECAY = 0.94;
    Vector2D predicted_pos(x, y);
    Vector2D current_vel(vx, vy);
    
    for (int i = 0; i < steps; ++i) {
        predicted_pos = predicted_pos + current_vel;
        current_vel = current_vel * BALL_DECAY;
    }
    
    return predicted_pos;
}

double InterceptEvaluator::evaluateCandidate(const InterceptCandidate& candidate, 
                                           const SimplePlayer& player,
                                           const SimpleBall& ball,
                                           const std::vector<SimplePlayer>& opponents,
                                           int opponent_min_step,
                                           int teammate_min_step) const {
    double score = 0.0;
    
    score += params_.shoot_spot_weight * calculateShootSpotValue(candidate.target_position);
    score -= params_.opponent_step_weight * calculateOpponentStepPenalty(candidate.steps_to_reach, opponent_min_step);
    score -= params_.teammate_step_weight * calculateTeammateStepPenalty(candidate.steps_to_reach, teammate_min_step);
    score -= params_.turn_penalty_weight * calculateTurnPenalty(candidate.turn_angle);
    score -= params_.distance_weight * calculateDistancePenalty(player.position().distance(candidate.target_position));
    score += params_.ball_speed_weight * calculateBallSpeedBonus(ball, candidate.steps_to_reach);
    
    return score;
}

double InterceptEvaluator::calculateShootSpotValue(const Vector2D& position) const {
    const Vector2D goal_center(52.5, 0.0);
    double distance_to_goal = position.distance(goal_center);
    
    if (distance_to_goal < 1.0) distance_to_goal = 1.0;
    
    double angle_factor = 1.0;
    if (position.x > 36.0) {
        double goal_angle = std::atan2(7.32, std::abs(52.5 - position.x));
        double ball_angle = std::atan2(std::abs(position.y), std::abs(52.5 - position.x));
        angle_factor = std::max(0.1, std::cos(ball_angle - goal_angle));
    }
    
    return (100.0 / distance_to_goal) * angle_factor;
}

double InterceptEvaluator::calculateOpponentStepPenalty(int self_step, int opponent_step) const {
    if (opponent_step >= 1000) return 0.0;
    
    int step_diff = self_step - opponent_step;
    if (step_diff <= 0) return 0.0;
    
    return step_diff * step_diff * 10.0;
}

double InterceptEvaluator::calculateTeammateStepPenalty(int self_step, int teammate_step) const {
    if (teammate_step >= 1000) return 0.0;
    
    int step_diff = self_step - teammate_step;
    if (step_diff <= 0) return 0.0;
    
    return step_diff * 2.0;
}

double InterceptEvaluator::calculateTurnPenalty(double turn_angle) const {
    return std::abs(turn_angle) * 5.0;
}

double InterceptEvaluator::calculateDistancePenalty(double distance) const {
    return distance * 2.0;
}

double InterceptEvaluator::calculateBallSpeedBonus(const SimpleBall& ball, int steps) const {
    double ball_speed = ball.velocity().length();
    if (steps <= 3 && ball_speed > 0.5) {
        return ball_speed * 10.0;
    }
    return 0.0;
}

BallInterceptor::BallInterceptor(const InterceptParams& params) 
    : params_(params), evaluator_(std::make_unique<InterceptEvaluator>()) {
}

BallInterceptor::~BallInterceptor() = default;

std::vector<InterceptCandidate> BallInterceptor::generateCandidates(const SimplePlayer& player, 
                                                                  const SimpleBall& ball,
                                                                  const std::vector<SimplePlayer>& opponents) const {
    std::vector<InterceptCandidate> candidates;
    
    for (int step = 1; step <= params_.max_predict_steps; ++step) {
        Vector2D ball_pos = ball.predictPosition(step);
        
        if (!isPathClear(player.position(), ball_pos, opponents, step)) {
            continue;
        }
        
        int steps_to_reach = calculateStepsToReach(player, ball_pos, opponents);
        if (steps_to_reach > step + 2) {
            continue;
        }
        
        InterceptCandidate candidate;
        candidate.target_position = ball_pos;
        candidate.steps_to_reach = steps_to_reach;
        candidate.turn_angle = calculateRequiredTurn(player, ball_pos);
        candidate.requires_turn = std::abs(candidate.turn_angle) > M_PI / 12.0;
        
        candidates.push_back(candidate);
        
        if (candidates.size() >= static_cast<size_t>(params_.max_candidates)) {
            break;
        }
    }
    
    return candidates;
}

InterceptCandidate BallInterceptor::getBestIntercept(const SimplePlayer& player, 
                                                   const SimpleBall& ball,
                                                   const std::vector<SimplePlayer>& opponents,
                                                   int opponent_min_step,
                                                   int teammate_min_step) const {
    auto candidates = generateCandidates(player, ball, opponents);
    
    if (candidates.empty()) {
        InterceptCandidate fallback;
        fallback.target_position = ball.predictPosition(params_.max_predict_steps);
        fallback.steps_to_reach = params_.max_predict_steps;
        return fallback;
    }
    
    InterceptCandidate best_candidate = candidates[0];
    double best_score = -std::numeric_limits<double>::max();
    
    for (auto& candidate : candidates) {
        candidate.evaluation_score = evaluator_->evaluateCandidate(candidate, player, ball, opponents, 
                                                                  opponent_min_step, teammate_min_step);
        
        if (candidate.evaluation_score > best_score) {
            best_score = candidate.evaluation_score;
            best_candidate = candidate;
        }
    }
    
    return best_candidate;
}

bool BallInterceptor::canIntercept(const SimplePlayer& player, const SimpleBall& ball, int max_steps) const {
    for (int step = 1; step <= max_steps; ++step) {
        Vector2D ball_pos = ball.predictPosition(step);
        int steps_to_reach = calculateStepsToReach(player, ball_pos, {});
        
        if (steps_to_reach <= step) {
            return true;
        }
    }
    return false;
}

Vector2D BallInterceptor::calculateAvoidanceForce(const SimplePlayer& player, 
                                                const std::vector<SimplePlayer>& opponents,
                                                const Vector2D& desired_direction) const {
    Vector2D avoidance_force(0, 0);
    const double influence_radius = params_.collision_radius * 2.0; // 2x collision radius for influence
    
    for (const auto& opponent : opponents) {
        Vector2D to_opponent = opponent.position() - player.position();
        double distance = to_opponent.length();
        
        if (distance < influence_radius && distance > 0.0) {
            Vector2D avoid_dir = to_opponent.normalized() * (-1.0);
            double force_magnitude = (influence_radius - distance) / influence_radius;
            avoidance_force = avoidance_force + avoid_dir * force_magnitude;
            cout << "🚨 [COLLISION AVOIDANCE] Distance: " << distance << ", Force: " << force_magnitude << endl;
        } else {
            // Reduced verbosity - only show when close
            if (distance < influence_radius * 2.0) {
                cout << "✅ [COLLISION CHECK] Distance: " << distance << " (safe)" << endl;
            }
        }
        
    }
    
    Vector2D final_direction = desired_direction + avoidance_force * 2.0; // Stronger avoidance
    return final_direction.normalized();
}

int BallInterceptor::calculateStepsToReach(const SimplePlayer& player, const Vector2D& target, 
                                         const std::vector<SimplePlayer>& opponents) const {
    double distance = player.position().distance(target);
    
    if (params_.enable_obstacle_avoidance && !opponents.empty()) {
        Vector2D desired_dir = (target - player.position()).normalized();
        Vector2D adjusted_dir = calculateAvoidanceForce(player, opponents, desired_dir);
        
        double detour_factor = 1.0 + (desired_dir - adjusted_dir).length() * 0.5;
        distance *= detour_factor;
    }
    
    double turn_steps = 0.0;
    double required_turn = std::abs(calculateRequiredTurn(player, target));
    if (required_turn > M_PI / 18.0) {
        turn_steps = required_turn / (M_PI / 6.0);
    }
    
    double move_steps = distance / params_.player_max_speed;
    
    return static_cast<int>(std::ceil(turn_steps + move_steps));
}


bool BallInterceptor::isPathClear(const Vector2D& start, const Vector2D& end, 
                                const std::vector<SimplePlayer>& opponents, int step_offset) const {
    if (!params_.enable_obstacle_avoidance || opponents.empty()) {
        cout << "[PATH CLEAR] No obstacles or avoidance disabled" << endl;
        return true;
    }
    
    Vector2D path_vector = end - start;
    double path_length = path_vector.length();
    
    cout << "[PATH CLEAR] Path length: " << path_length << ", collision radius: " << params_.collision_radius << endl;
    
    if (path_length < 0.01) {
        cout << "[PATH CLEAR] Path too short" << endl;
        return true;
    }
    
    Vector2D path_dir = path_vector.normalized();
    const int check_points = std::max(3, static_cast<int>(path_length / 0.5));
    
    cout << "[PATH CLEAR] Checking " << check_points << " points along path" << endl;
    
    for (int i = 1; i < check_points; ++i) {
        Vector2D check_point = start + path_dir * (path_length * i / check_points);
        
        for (const auto& opponent : opponents) {
            double dist = check_point.distance(opponent.position());
            cout << "[PATH CLEAR] Point " << i << " distance to opponent: " << dist << endl;
            if (dist < params_.collision_radius) {
                cout << "[PATH CLEAR] BLOCKED at point " << i << endl;
                return false;
            }
        }
    }
    
    cout << "[PATH CLEAR] Path is clear" << endl;
    return true;
}

Vector2D BallInterceptor::findOptimalPath(const SimplePlayer& player, const Vector2D& target,
                                        const std::vector<SimplePlayer>& opponents) const {
    cout << "[OPTIMAL PATH] Checking path from (" << player.position().x << ", " << player.position().y 
         << ") to (" << target.x << ", " << target.y << ")" << endl;
    cout << "[OPTIMAL PATH] Opponents: " << opponents.size() << ", Avoidance enabled: " << params_.enable_obstacle_avoidance << endl;
    
    bool path_clear = isPathClear(player.position(), target, opponents);
    cout << "[OPTIMAL PATH] Path clear: " << (path_clear ? "YES" : "NO") << endl;
    
    if (!params_.enable_obstacle_avoidance || opponents.empty() || path_clear) {
        cout << "[OPTIMAL PATH] Using direct path to target" << endl;
        return target;
    }
    
    cout << "[OPTIMAL PATH] Path blocked - calculating avoidance" << endl;
    Vector2D direct_dir = (target - player.position()).normalized();
    Vector2D adjusted_dir = calculateAvoidanceForce(player, opponents, direct_dir);
    
    double step_size = player.position().distance(target) * 0.7;
    Vector2D intermediate_target = player.position() + adjusted_dir * step_size;
    
    cout << "[OPTIMAL PATH] Adjusted target: (" << intermediate_target.x << ", " << intermediate_target.y << ")" << endl;
    return intermediate_target;
}

double BallInterceptor::calculateRequiredTurn(const SimplePlayer& player, const Vector2D& target) const {
    Vector2D to_target = target - player.position();
    double target_angle = std::atan2(to_target.y, to_target.x);
    
    double angle_diff = target_angle - player.angle;
    
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
    
    return angle_diff;
}

BallInterceptAction::BallInterceptAction() 
    : interceptor_(std::make_unique<BallInterceptor>()) {
}

BallInterceptAction::BallInterceptAction(const BallInterceptor::InterceptParams& params) 
    : interceptor_(std::make_unique<BallInterceptor>(params)) {
}

BallInterceptAction::~BallInterceptAction() = default;

BallInterceptAction::Action BallInterceptAction::execute(const SimplePlayer& player, 
                                                       const SimpleBall& ball,
                                                       const std::vector<SimplePlayer>& opponents,
                                                       const std::vector<SimplePlayer>& teammates,
                                                       int opponent_min_step,
                                                       int teammate_min_step) {
    
    InterceptCandidate best_intercept = interceptor_->getBestIntercept(player, ball, opponents, 
                                                                     opponent_min_step, teammate_min_step);
    
    if (best_intercept.steps_to_reach <= 0) {
        return Action(WAIT);
    }
    
    double distance_to_target = player.position().distance(best_intercept.target_position);
    
    if (distance_to_target < 0.5) {
        return Action(WAIT);
    }
    
    if (best_intercept.requires_turn && std::abs(best_intercept.turn_angle) > M_PI / 8.0) {
        return createTurnAction(player, best_intercept.target_position);
    }
    
    if (best_intercept.requires_turn) {
        return Action(TURN_AND_DASH, 
                     calculateOptimalDashPower(player, best_intercept.target_position, distance_to_target, best_intercept.steps_to_reach),
                     best_intercept.turn_angle, Vector2D(), best_intercept.target_position);
    }
    
    return createDashAction(player, best_intercept.target_position, opponents);
}

bool BallInterceptAction::shouldChase(const SimplePlayer& player, 
                                    const SimpleBall& ball,
                                    int self_min_step,
                                    int teammate_min_step, 
                                    int opponent_min_step,
                                    int pressing_factor) const {
    
    if (self_min_step <= 3) {
        return true;
    }
    
    if (teammate_min_step < 1000 && self_min_step > teammate_min_step) {
        return false;
    }
    
    if (opponent_min_step < 1000) {
        int adjusted_opponent_step = opponent_min_step + pressing_factor;
        return self_min_step <= adjusted_opponent_step;
    }
    
    return true;
}

int BallInterceptAction::calculateMinInterceptSteps(const SimplePlayer& player, const SimpleBall& ball) const {
    int min_steps = 1000;
    
    for (int step = 1; step <= 20; ++step) {
        Vector2D ball_pos = ball.predictPosition(step);
        double distance = player.position().distance(ball_pos);
        
        double required_turn = std::abs(interceptor_->calculateRequiredTurn(player, ball_pos));
        int turn_steps = (required_turn > M_PI / 18.0) ? static_cast<int>(required_turn / (M_PI / 6.0)) : 0;
        
        int move_steps = static_cast<int>(std::ceil(distance / 1.0));
        int total_steps = turn_steps + move_steps;
        
        if (total_steps <= step) {
            min_steps = std::min(min_steps, total_steps);
        }
    }
    
    return (min_steps == 1000) ? 50 : min_steps;
}

BallInterceptAction::Action BallInterceptAction::createDashAction(const SimplePlayer& player, 
                                                                const Vector2D& target, 
                                                                const std::vector<SimplePlayer>& opponents) {
    Vector2D optimal_target = interceptor_->findOptimalPath(player, target, opponents);
    Vector2D direction = (optimal_target - player.position()).normalized();
    double distance = player.position().distance(optimal_target);
    
    double power = calculateOptimalDashPower(player, optimal_target, distance, 
                                           static_cast<int>(distance / 1.0) + 1);
    
    return Action(DASH, power, 0.0, direction, optimal_target);
}

BallInterceptAction::Action BallInterceptAction::createTurnAction(const SimplePlayer& player, 
                                                                const Vector2D& target) {
    double required_turn = interceptor_->calculateRequiredTurn(player, target);
    
    double turn_power = std::min(100.0, std::abs(required_turn) * 180.0 / M_PI * 2.0);
    
    return Action(TURN, turn_power, required_turn, Vector2D(), target);
}

double BallInterceptAction::calculateOptimalDashPower(const SimplePlayer& player, 
                                                    const Vector2D& target, 
                                                    double distance, 
                                                    int steps) const {
    if (steps <= 0) steps = 1;
    
    double required_speed = distance / steps;
    double speed_ratio = required_speed / 1.0;
    
    double base_power = std::min(100.0, speed_ratio * 100.0);
    
    if (distance < 2.0) {
        base_power *= 1.2;
    }
    
    return std::max(10.0, std::min(100.0, base_power));
}

}