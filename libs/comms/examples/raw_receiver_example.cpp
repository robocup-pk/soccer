#include "VisionTypes.h"
#include <iostream>
#include <signal.h>
#include <iomanip>
#include <thread>
#include <chrono>

bool running = true;

void handleSignal(int) { running = false; }

int main() {
  signal(SIGINT, handleSignal);

  std::cout << "=== Raw UDP Data Receiver ===" << std::endl;
  std::cout << "This example receives raw UDP packets without parsing protobuf." << std::endl;
  std::cout << "Listening on SSL-Vision multicast address..." << std::endl;

  // Create raw data receiver for SSL-Vision multicast
  comm::RawDataReceiver receiver("224.5.23.2", 10006);

  // Set up callback to process received data
  receiver.setRawCallback([](const void* data, size_t size) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") << "] "
              << "Received " << size << " bytes: ";

    // Try to display as printable text first
    const char* str_data = static_cast<const char*>(data);
    bool is_printable = true;
    size_t print_size = std::min(size, size_t(50));

    for (size_t i = 0; i < print_size; ++i) {
      if (str_data[i] < 32 || str_data[i] > 126) {
        is_printable = false;
        break;
      }
    }

    if (is_printable && size > 10) {
      // If it looks like text, show it as text
      std::cout << "\"" << std::string(str_data, print_size) << "\"";
      if (size > 50) std::cout << "...";
    } else {
      // Show as hex (this is what SSL-Vision protobuf data looks like)
      std::cout << "0x";
      for (size_t i = 0; i < print_size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(static_cast<unsigned char>(str_data[i]));
      }
      if (size > 50) std::cout << "...";
      std::cout << std::dec;  // Reset to decimal
    }
    std::cout << std::endl;
  });

  if (!receiver.start()) {
    std::cerr << "Failed to start receiver. Make sure:" << std::endl;
    std::cerr << "1. You have permission to bind to the multicast address" << std::endl;
    std::cerr << "2. No other process is using port 10006" << std::endl;
    return 1;
  }

  std::cout << "Receiving raw data on 224.5.23.2:10006" << std::endl;
  std::cout << "Press Ctrl+C to exit..." << std::endl;

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  std::cout << "\nShutting down..." << std::endl;
  receiver.stop();
  return 0;
}
