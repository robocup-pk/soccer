// C++ standard libraries
#include <iostream>
#include <chrono>
#include <csignal>  // For signal handling
#include <thread>

// self libs
#include "RRT.h"

bool keep_running = true;

void HandleSignal(int) {
  keep_running = false;
}

int main(int argc, char* argv[]) {
  // Register the signal handler
  std::signal(SIGINT, HandleSignal);

  auto start_time = std::chrono::high_resolution_clock::now();
  auto last_time = start_time;

  // Main loop
  while (keep_running) {
    std::cout << "x_dist: " << algos::x_distribution(algos::rng) << std::endl;
    // Optionally sleep to avoid flooding stdout
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "Exited loop.\n";
  return 0;
}
