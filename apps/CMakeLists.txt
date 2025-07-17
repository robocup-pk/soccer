add_executable(algos_engine AlgosEngine.cpp)
target_link_libraries(algos_engine PRIVATE state_lib vis_lib cfg_lib algos_lib rob_lib)

add_executable(soccer_manual_game SoccerManualGame.cpp)
target_link_libraries(soccer_manual_game PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib)

add_executable(soccer_engine SoccerEngine.cpp)
target_link_libraries(soccer_engine PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib ref_lib)

add_executable(soccer_team_manual SoccerTeamManual.cpp)
target_link_libraries(soccer_team_manual PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib ref_lib)