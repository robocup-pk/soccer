#include <gtest/gtest.h>
#include <thread>
#include <chrono>

#include "Estimator.h"

TEST(EstimatorTest, TestEncoders) {
  est::Estimator estimator;

  double ticks_interval_ms = 10;
  double ticks_per_second = 1000 / ticks_interval_ms;
  auto start = high_resolution_clock::now();

  for (int i = 0; i < 1000; i++) {
    // Ground Truth
    int ticks = i;

    auto end = high_resolution_clock::now();
    duration<double> elapsed = end - start;
    double rpm = (ticks / hw::Config::ticks_per_rev) * (60 / elapsed_time);

    std::cout << "Ticks: " << ticks << " . Rpm: " << rpm << std::endl;

    // Estimation

    std::this_thread::sleep_for(std::chrono::milliseconds(ticks_interval_ms)); // Sleep for 10ms
  }
}
