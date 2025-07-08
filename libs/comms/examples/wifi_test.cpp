#include "UdpWifi.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <csignal>

bool running = true;

void signalHandler(int) { running = false; }

int main() {
  std::cout << "Starting UDP WiFi receiver on port 4210...\n";
  std::cout << "Press Ctrl+C to exit.\n\n";

  std::signal(SIGINT, signalHandler);

  comm::UDPWiFi receiver(4210);
  if (!receiver.start()) {
    std::cerr << "Failed to start UDP receiver.\n";
    return 1;
  }

  while (running) {
    int16_t x = receiver.getX();
    int16_t y = receiver.getY();
    std::cout << "X: " << x << "  Y: " << y << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "\nStopping receiver...\n";
  receiver.stop();
  std::cout << "Done.\n";

  return 0;
}
