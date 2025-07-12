#include <iostream>
#include <thread>
#include <chrono>
#include "VisionManager.h"

int main() {
  // Create and start the manager (multicast addr + port optional)
  comm::VisionManager mgr("224.5.23.2", 10006);
  // mgr.start();  // not needed, ctor already calls start()

  std::cout << "Waiting 2 seconds to collect some data...\n";
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Fetch yellow robots
  auto yellow = mgr.getYellowRobots();
  std::cout << "Yellow robots (" << yellow.size() << "):\n";
  for (const auto &r : yellow) {
    std::cout << "  id=" << r.id << " x=" << r.x << " y=" << r.y << " θ=" << r.orientation << "\n";
  }

  // Fetch blue robots
  auto blue = mgr.getBlueRobots();
  std::cout << "Blue robots (" << blue.size() << "):\n";
  for (const auto &r : blue) {
    std::cout << "  id=" << r.id << " x=" << r.x << " y=" << r.y << " θ=" << r.orientation << "\n";
  }

  // Cleanly stop
  mgr.stop();
  std::cout << "Done.\n";
  return 0;
}