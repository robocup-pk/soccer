#ifndef TEAM_COMPETITION_H
#define TEAM_COMPETITION_H

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "GameObject.h"
#include "MultiPlayerIntelligentMovement2.h"
#include "MultiPlayerRRT.h"

namespace vis {

/**
 * TeamCompetition - Two-Team AI Competition System
 * 
 * Manages two teams of robots with different AI strategies:
 * - Team A: Can use either MultiPlayerIntelligentMovement2 or MultiPlayerRRT
 * - Team B: Can use either MultiPlayerIntelligentMovement2 or MultiPlayerRRT
 * 
 * This allows testing different strategy combinations:
 * - Ball Intercept vs Ball Intercept
 * - Ball Intercept vs RRT  
 * - RRT vs RRT
 */
class TeamCompetition {
public:
    enum TeamStrategy {
        BALL_INTERCEPT = 1,  // Use MultiPlayerIntelligentMovement2
        RRT_NAVIGATION = 2   // Use MultiPlayerRRT
    };

    TeamCompetition();
    ~TeamCompetition();

    // Team configuration
    void SetTeamAPlayers(const std::vector<std::string>& player_names);
    void SetTeamBPlayers(const std::vector<std::string>& player_names);
    void SetTeamAStrategy(TeamStrategy strategy);
    void SetTeamBStrategy(TeamStrategy strategy);
    void SetBallName(const std::string& name) { ball_name_ = name; }
    
    // Movement configuration
    void SetMaxSpeed(double speed);
    void SetMaxAcceleration(double accel);
    void SetAutoMode(bool enabled);
    
    // Individual team configuration
    void SetTeamAMaxSpeed(double speed);
    void SetTeamBMaxSpeed(double speed);
    void SetTeamAEnabled(bool enabled);
    void SetTeamBEnabled(bool enabled);
    
    // Main update method
    void UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt);
    
    // Toggle methods
    void ToggleAutoMode() { auto_mode_enabled_ = !auto_mode_enabled_; }
    void ToggleTeamA();
    void ToggleTeamB();
    bool IsAutoModeEnabled() const { return auto_mode_enabled_; }
    
    // Status methods
    std::vector<std::string> GetTeamAPlayers() const { return team_a_players_; }
    std::vector<std::string> GetTeamBPlayers() const { return team_b_players_; }
    TeamStrategy GetTeamAStrategy() const { return team_a_strategy_; }
    TeamStrategy GetTeamBStrategy() const { return team_b_strategy_; }
    void PrintStatus() const;
    
    // Quick setup methods for common configurations
    void SetupDefaultTeams(); // 5v5 setup with robots 0-4 vs 5-9
    void SetupBallInterceptVsRRT(); // Team A uses ball intercept, Team B uses RRT
    void SetupRRTVsBallIntercept(); // Team A uses RRT, Team B uses ball intercept
    void SetupRRTVsRRT(); // Both teams use RRT
    void SetupBallInterceptVsBallIntercept(); // Both teams use ball intercept

private:
    // Team management systems
    std::unique_ptr<MultiPlayerIntelligentMovement2> team_a_ball_intercept_;
    std::unique_ptr<MultiPlayerRRT> team_a_rrt_;
    std::unique_ptr<MultiPlayerIntelligentMovement2> team_b_ball_intercept_;
    std::unique_ptr<MultiPlayerRRT> team_b_rrt_;
    
    // Configuration
    std::vector<std::string> team_a_players_;
    std::vector<std::string> team_b_players_;
    TeamStrategy team_a_strategy_;
    TeamStrategy team_b_strategy_;
    std::string ball_name_;
    bool auto_mode_enabled_;
    bool team_a_enabled_;
    bool team_b_enabled_;
    
    // Helper methods
    void InitializeTeamA();
    void InitializeTeamB();
    void UpdateTeamA(std::map<std::string, GameObject>& game_objects, float dt);
    void UpdateTeamB(std::map<std::string, GameObject>& game_objects, float dt);
    void ConfigureTeamSystem(MultiPlayerIntelligentMovement2* system, 
                           const std::vector<std::string>& team_players,
                           const std::vector<std::string>& opponent_players);
    void ConfigureTeamSystem(MultiPlayerRRT* system,
                           const std::vector<std::string>& team_players,
                           const std::vector<std::string>& opponent_players);
    std::string GetStrategyName(TeamStrategy strategy) const;
};

} // namespace vis

#endif // TEAM_COMPETITION_H