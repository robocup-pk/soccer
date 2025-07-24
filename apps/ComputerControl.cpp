// // // cpp std libs
// // #include <algorithm>
// // #include <iostream>
// // #include <chrono>
// // #include <vector>
// // #include <cmath>

// // // self libs
// // #include "GLSimulation.h"
// // #include "SoccerObject.h"
// // #include "Kinematics.h"
// // #include "Utils.h"
// // #include "Coordinates.h"
// // #include "RobotManager.h"
// // #include "SoccerField.h"
// // #include "AutoRef.h"
// // #include "Team.h"

// // int main(int argc, char* argv[]) {
// //   // TODO: Game State

// //   std::vector<rob::RobotManager> team_one_robot_managers(cfg::SystemConfig::num_robots);
// //   std::vector<rob::RobotManager> team_two_robot_managers(cfg::SystemConfig::num_robots);

// //   state::Team team_one = state::Team(1, cfg::SystemConfig::num_robots,
// team_one_robot_managers);
// //   state::Team team_two = state::Team(2, cfg::SystemConfig::num_robots,
// team_two_robot_managers);

// //   // Simulation - OpenGL
// //   vis::GLSimulation gl_simulation;
// //   std::vector<state::SoccerObject>& team_one_soccer_objects = team_one.GetSoccerObjects();
// //   std::vector<state::SoccerObject>& team_two_soccer_objects = team_two.GetSoccerObjects();
// //   gl_simulation.InitGameObjects(team_one_soccer_objects, team_two_soccer_objects);
// //   GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
// //   glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

// //   while (1) {
// //     float dt = util::CalculateDt();

// //     vis::ProcessInput(gl_window, team_one_robot_managers, team_two_robot_managers);

// //     // Update SoccerObjects using robot managers
// //     team_one.UpdateSoccerObjects();
// //     team_two.UpdateSoccerObjects();

// //     // Step 1: Use SoccerObjects to Compute Plan
// //     // std::vector<std::vector<Eigen::Vector3d>> plan = algos::ComputePlan(soccer_objects);

// //     // Step 2: Send plan to robot managers
// //     // for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
// //     //   robot_managers[i].SetPath(plan[i]);
// //     // }

// //     // Referee
// //     // Step: Rule checking on current soccer objects
// //     // 1. Kinematics (Collisions, Max speeds)
// //     // kin::CheckAndResolveCollisions(soccer_objects);
// //     ref::CheckCollisions(team_one_soccer_objects);
// //     ref::CheckCollisions(team_two_soccer_objects);
// //     ref::CheckForGoals(team_one_soccer_objects);
// //     ref::CheckForGoals(team_two_soccer_objects);
// //     // 2.
// //     // 3.

// //     team_one_soccer_objects[team_one_soccer_objects.size() - 1].Move(dt);  // Move the ball
// //     team_two_soccer_objects[team_two_soccer_objects.size() - 1].Move(dt);
// //     // Step: Simulation
// //     if (!gl_simulation.RunSimulationStep(team_one_soccer_objects, team_two_soccer_objects,
// dt))
// //     {
// //       std::cout << "[main] Simulation finished" << std::endl;
// //       break;
// //     }
// //   }

// //   return 0;
// // }

// #include <algorithm>
// #include <iostream>
// #include <chrono>
// #include <vector>
// #include <cmath>

// // self libs
// #include "GLSimulation.h"
// #include "SoccerObject.h"
// #include "Kinematics.h"
// #include "Utils.h"
// #include "Coordinates.h"
// #include "RobotManager.h"
// #include "SoccerField.h"
// #include "AutoRef.h"

// int main(int argc, char* argv[]) {
//   // TODO: Game State
//   std::vector<state::SoccerObject> team_one_soccer_objects;
//   std::vector<state::SoccerObject> team_two_soccer_objects;
//   state::InitSoccerObjects(team_one_soccer_objects);
//   state::InitSoccerObjects(team_two_soccer_objects);

//   // Simulation - OpenGL
//   vis::GLSimulation gl_simulation;
//   gl_simulation.InitGameObjects(team_one_soccer_objects);
//   gl_simulation.InitGameObjects(team_two_soccer_objects);

//   GLFWwindow* gl_window = gl_simulation.GetRawGLFW();
//   glfwSetMouseButtonCallback(gl_window, vis::MouseButtonCallback);

//   // Robot(s)
//   std::vector<rob::RobotManager> team_one_robot_managers(cfg::SystemConfig::num_robots / 2);
//   std::vector<rob::RobotManager> team_two_robot_managers(cfg::SystemConfig::num_robots / 2);

//   // Initializing Robot Managers Positions using SoccerObjects to avoid conflicts
//   for (int i = 0; i < cfg::SystemConfig::num_robots / 2; ++i) {
//     team_one_robot_managers[i].InitializePose(team_one_soccer_objects[i].position);
//     team_two_robot_managers[i].InitializePose(team_one_soccer_objects[i].position);
//   }

//   while (1) {
//     float dt = util::CalculateDt();

//     vis::ProcessInputTwoTeams(gl_window, team_one_robot_managers, team_two_robot_managers);

//     // Update SoccerObjects using robot managers
//     for (int i = 0; i < cfg::SystemConfig::num_robots / 2; ++i) {
//       team_one_soccer_objects[i] = team_one_robot_managers[i];
//       team_two_soccer_objects[i] = team_two_robot_managers[i];
//     }

//     // Step 1: Use SoccerObjects to Compute Plan
//     // std::vector<std::vector<Eigen::Vector3d>> plan = algos::ComputePlan(soccer_objects);

//     // Step 2: Send plan to robot managers
//     // for (int i = 0; i < cfg::SystemConfig::num_robots; ++i) {
//     //   robot_managers[i].SetPath(plan[i]);
//     // }

//     // Referee
//     // Step: Rule checking on current soccer objects
//     // 1. Kinematics (Collisions, Max speeds)
//     // kin::CheckAndResolveCollisions(soccer_objects);

//     // uncomment soon
//     // ref::CheckCollisions(soccer_objects);
//     // ref::CheckForGoals(soccer_objects);

//     // 2.
//     // 3.

//     team_one_soccer_objects[team_one_soccer_objects.size() - 1].Move(dt);  // Move the ball
//     // team_two_soccer_objects[team_two_soccer_objects.size() - 1].Move(dt);
//     // Step: Simulation
//     if (!gl_simulation.RunSimulationStepTwoTeams(team_one_soccer_objects,
//     team_two_soccer_objects,
//                                                  dt)) {
//       std::cout << "[main] Simulation finished" << std::endl;
//       break;
//     }
//   }

//   return 0;
// }
