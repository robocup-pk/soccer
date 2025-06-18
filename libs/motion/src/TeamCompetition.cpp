#include "TeamCompetition.h"
#include <iostream>
#include <algorithm>

namespace vis {

TeamCompetition::TeamCompetition()
    : ball_name_("ball"),
      team_a_strategy_(BALL_INTERCEPT),
      team_b_strategy_(RRT_NAVIGATION),
      auto_mode_enabled_(true),
      team_a_enabled_(true),
      team_b_enabled_(true) {
    
    // Initialize all team systems
    team_a_ball_intercept_ = std::make_unique<MultiPlayerIntelligentMovement2>();
    team_a_rrt_ = std::make_unique<MultiPlayerRRT>();
    team_b_ball_intercept_ = std::make_unique<MultiPlayerIntelligentMovement2>();
    team_b_rrt_ = std::make_unique<MultiPlayerRRT>();
    
    // Set up default 5v5 teams
    SetupDefaultTeams();
    
    std::cout << "[TeamCompetition] Initialized two-team competition system" << std::endl;
    PrintStatus();
}

TeamCompetition::~TeamCompetition() = default;

void TeamCompetition::SetupDefaultTeams() {
    // Team A: robots 0-4
    // SetTeamAPlayers({"robot0", "robot1", "robot2", "robot3", "robot4"});
    // // Team B: robots 5-9  
    // SetTeamBPlayers({"robot5", "robot6", "robot7", "robot8", "robot9"});
    SetTeamAPlayers({"robot0"});
    SetTeamBPlayers({"robot5"});
    
    // Default: Team A uses ball intercept, Team B uses RRT
    SetTeamAStrategy(BALL_INTERCEPT);
    SetTeamBStrategy(RRT_NAVIGATION);
    
    std::cout << "[TeamCompetition] Default 5v5 setup: Team A (Ball Intercept) vs Team B (RRT)" << std::endl;
}

void TeamCompetition::SetupBallInterceptVsRRT() {
    SetTeamAStrategy(BALL_INTERCEPT);
    SetTeamBStrategy(RRT_NAVIGATION);
    std::cout << "[TeamCompetition] Setup: Ball Intercept vs RRT" << std::endl;
}

void TeamCompetition::SetupRRTVsBallIntercept() {
    SetTeamAStrategy(RRT_NAVIGATION);
    SetTeamBStrategy(BALL_INTERCEPT);
    std::cout << "[TeamCompetition] Setup: RRT vs Ball Intercept" << std::endl;
}

void TeamCompetition::SetupRRTVsRRT() {
    SetTeamAStrategy(RRT_NAVIGATION);
    SetTeamBStrategy(RRT_NAVIGATION);
    std::cout << "[TeamCompetition] Setup: RRT vs RRT" << std::endl;
}

void TeamCompetition::SetupBallInterceptVsBallIntercept() {
    SetTeamAStrategy(BALL_INTERCEPT);
    SetTeamBStrategy(BALL_INTERCEPT);
    std::cout << "[TeamCompetition] Setup: Ball Intercept vs Ball Intercept" << std::endl;
}

void TeamCompetition::SetTeamAPlayers(const std::vector<std::string>& player_names) {
    team_a_players_ = player_names;
    InitializeTeamA();
    std::cout << "[TeamCompetition] Team A players: ";
    for (const auto& name : team_a_players_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
}

void TeamCompetition::SetTeamBPlayers(const std::vector<std::string>& player_names) {
    team_b_players_ = player_names;
    InitializeTeamB();
    std::cout << "[TeamCompetition] Team B players: ";
    for (const auto& name : team_b_players_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
}

void TeamCompetition::SetTeamAStrategy(TeamStrategy strategy) {
    team_a_strategy_ = strategy;
    InitializeTeamA();
    std::cout << "[TeamCompetition] Team A strategy: " << GetStrategyName(strategy) << std::endl;
}

void TeamCompetition::SetTeamBStrategy(TeamStrategy strategy) {
    team_b_strategy_ = strategy;
    InitializeTeamB();
    std::cout << "[TeamCompetition] Team B strategy: " << GetStrategyName(strategy) << std::endl;
}

void TeamCompetition::InitializeTeamA() {
    if (team_a_players_.empty()) return;
    
    // Configure the appropriate system for Team A
    if (team_a_strategy_ == BALL_INTERCEPT) {
        ConfigureTeamSystem(team_a_ball_intercept_.get(), team_a_players_, team_b_players_);
        team_a_ball_intercept_->SetAutoMode(team_a_enabled_);
        team_a_rrt_->SetAutoMode(false); // Disable the other system
    } else {
        ConfigureTeamSystem(team_a_rrt_.get(), team_a_players_, team_b_players_);
        team_a_rrt_->SetAutoMode(team_a_enabled_);
        team_a_ball_intercept_->SetAutoMode(false); // Disable the other system
    }
}

void TeamCompetition::InitializeTeamB() {
    if (team_b_players_.empty()) return;
    
    // Configure the appropriate system for Team B
    if (team_b_strategy_ == BALL_INTERCEPT) {
        ConfigureTeamSystem(team_b_ball_intercept_.get(), team_b_players_, team_a_players_);
        team_b_ball_intercept_->SetAutoMode(team_b_enabled_);
        team_b_rrt_->SetAutoMode(false); // Disable the other system
    } else {
        ConfigureTeamSystem(team_b_rrt_.get(), team_b_players_, team_a_players_);
        team_b_rrt_->SetAutoMode(team_b_enabled_);
        team_b_ball_intercept_->SetAutoMode(false); // Disable the other system
    }
}

void TeamCompetition::ConfigureTeamSystem(MultiPlayerIntelligentMovement2* system,
                                         const std::vector<std::string>& team_players,
                                         const std::vector<std::string>& opponent_players) {
    if (!system) return;
    
    // Clear ALL existing players first (including defaults like robot0)
    auto current_players = system->GetControlledPlayers();
    for (const auto& player : current_players) {
        system->RemoveControlledPlayer(player);
    }
    
    // Add new team players
    for (const auto& player : team_players) {
        system->AddControlledPlayer(player);
        std::cout << "[TeamCompetition] Added Ball Intercept player: " << player << std::endl;
    }
    
    // Set opponents (other team)
    system->SetOpponents(opponent_players);
    system->SetBallName(ball_name_);
}

void TeamCompetition::ConfigureTeamSystem(MultiPlayerRRT* system,
                                         const std::vector<std::string>& team_players,
                                         const std::vector<std::string>& opponent_players) {
    if (!system) return;
    
    // Clear ALL existing players first (including defaults like robot0)
    auto current_players = system->GetControlledPlayers();
    for (const auto& player : current_players) {
        system->RemoveControlledPlayer(player);
    }
    
    // Add new team players
    for (const auto& player : team_players) {
        system->AddControlledPlayer(player);
        std::cout << "[TeamCompetition] Added RRT player: " << player << std::endl;
    }
    
    // Set opponents (other team)
    system->SetOpponents(opponent_players);
    system->SetBallName(ball_name_);
}

void TeamCompetition::UpdateMovement(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!auto_mode_enabled_) return;
    
    // Update Team A
    UpdateTeamA(game_objects, dt);
    
    // Update Team B
    UpdateTeamB(game_objects, dt);
}

void TeamCompetition::UpdateTeamA(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!team_a_enabled_) return;
    
    if (team_a_strategy_ == BALL_INTERCEPT && team_a_ball_intercept_) {
        team_a_ball_intercept_->UpdateMovement(game_objects, dt);
    } else if (team_a_strategy_ == RRT_NAVIGATION && team_a_rrt_) {
        team_a_rrt_->UpdateMovement(game_objects, dt);
    }
}

void TeamCompetition::UpdateTeamB(std::map<std::string, GameObject>& game_objects, float dt) {
    if (!team_b_enabled_) return;
    
    if (team_b_strategy_ == BALL_INTERCEPT && team_b_ball_intercept_) {
        team_b_ball_intercept_->UpdateMovement(game_objects, dt);
    } else if (team_b_strategy_ == RRT_NAVIGATION && team_b_rrt_) {
        team_b_rrt_->UpdateMovement(game_objects, dt);
    }
}

void TeamCompetition::SetMaxSpeed(double speed) {
    if (team_a_ball_intercept_) team_a_ball_intercept_->SetMaxSpeed(speed);
    if (team_a_rrt_) team_a_rrt_->SetMaxSpeed(speed);
    if (team_b_ball_intercept_) team_b_ball_intercept_->SetMaxSpeed(speed);
    if (team_b_rrt_) team_b_rrt_->SetMaxSpeed(speed);
}

void TeamCompetition::SetMaxAcceleration(double accel) {
    if (team_a_ball_intercept_) team_a_ball_intercept_->SetMaxAcceleration(accel);
    if (team_a_rrt_) team_a_rrt_->SetMaxAcceleration(accel);
    if (team_b_ball_intercept_) team_b_ball_intercept_->SetMaxAcceleration(accel);
    if (team_b_rrt_) team_b_rrt_->SetMaxAcceleration(accel);
}

void TeamCompetition::SetTeamAMaxSpeed(double speed) {
    if (team_a_ball_intercept_) team_a_ball_intercept_->SetMaxSpeed(speed);
    if (team_a_rrt_) team_a_rrt_->SetMaxSpeed(speed);
}

void TeamCompetition::SetTeamBMaxSpeed(double speed) {
    if (team_b_ball_intercept_) team_b_ball_intercept_->SetMaxSpeed(speed);
    if (team_b_rrt_) team_b_rrt_->SetMaxSpeed(speed);
}

void TeamCompetition::SetAutoMode(bool enabled) {
    auto_mode_enabled_ = enabled;
    InitializeTeamA();
    InitializeTeamB();
}

void TeamCompetition::SetTeamAEnabled(bool enabled) {
    team_a_enabled_ = enabled;
    if (team_a_strategy_ == BALL_INTERCEPT && team_a_ball_intercept_) {
        team_a_ball_intercept_->SetAutoMode(enabled && auto_mode_enabled_);
    } else if (team_a_strategy_ == RRT_NAVIGATION && team_a_rrt_) {
        team_a_rrt_->SetAutoMode(enabled && auto_mode_enabled_);
    }
}

void TeamCompetition::SetTeamBEnabled(bool enabled) {
    team_b_enabled_ = enabled;
    if (team_b_strategy_ == BALL_INTERCEPT && team_b_ball_intercept_) {
        team_b_ball_intercept_->SetAutoMode(enabled && auto_mode_enabled_);
    } else if (team_b_strategy_ == RRT_NAVIGATION && team_b_rrt_) {
        team_b_rrt_->SetAutoMode(enabled && auto_mode_enabled_);
    }
}

void TeamCompetition::ToggleTeamA() {
    SetTeamAEnabled(!team_a_enabled_);
    std::cout << "[TeamCompetition] Team A " << (team_a_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
}

void TeamCompetition::ToggleTeamB() {
    SetTeamBEnabled(!team_b_enabled_);
    std::cout << "[TeamCompetition] Team B " << (team_b_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
}

std::string TeamCompetition::GetStrategyName(TeamStrategy strategy) const {
    switch (strategy) {
        case BALL_INTERCEPT: return "Ball Intercept";
        case RRT_NAVIGATION: return "RRT Navigation";
        default: return "Unknown";
    }
}

void TeamCompetition::PrintStatus() const {
    std::cout << "[TeamCompetition] === COMPETITION STATUS ===" << std::endl;
    std::cout << "  Auto mode: " << (auto_mode_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "  Ball: " << ball_name_ << std::endl;
    
    std::cout << "  Team A (" << (team_a_enabled_ ? "ON" : "OFF") << "): " 
              << GetStrategyName(team_a_strategy_) << std::endl;
    std::cout << "    Players: ";
    for (const auto& name : team_a_players_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    
    std::cout << "  Team B (" << (team_b_enabled_ ? "ON" : "OFF") << "): " 
              << GetStrategyName(team_b_strategy_) << std::endl;
    std::cout << "    Players: ";
    for (const auto& name : team_b_players_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    
    std::cout << "  Competition: " << GetStrategyName(team_a_strategy_) 
              << " vs " << GetStrategyName(team_b_strategy_) << std::endl;
}

} // namespace vis