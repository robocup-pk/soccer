add_executable(algos_engine AlgosEngine.cpp)
target_link_libraries(algos_engine PRIVATE state_lib vis_lib cfg_lib algos_lib)

add_executable(soccer_manual_game SoccerManualGame.cpp)
target_link_libraries(soccer_manual_game PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib)

add_executable(soccer_engine SoccerEngine.cpp)
target_link_libraries(soccer_engine PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib ref_lib)

#add_executable(computer_control ComputerControl.cpp)
#target_link_libraries(computer_control PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib ref_lib)

add_executable(play TwoTeams.cpp)
target_link_libraries(play PRIVATE state_lib vis_lib cfg_lib rob_lib utils_lib ref_lib)