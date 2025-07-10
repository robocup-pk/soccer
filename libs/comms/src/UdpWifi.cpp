#include "UdpWifi.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>  // Add this include
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

namespace comm {

UDPWiFi::UDPWiFi(uint16_t port)
    : _sockfd(-1), _port(port), _running(false), _x(0), _y(0), _btn(0) {}

UDPWiFi::~UDPWiFi() { stop(); }

bool UDPWiFi::start() {
  _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (_sockfd < 0) {
    return false;
  }

  // Set socket to non-blocking
  fcntl(_sockfd, F_SETFL, O_NONBLOCK);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(_port);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(_sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(_sockfd);
    _sockfd = -1;
    return false;
  }

  _running = true;
  _thread = std::thread(&UDPWiFi::listenLoop, this);
  return true;
}

void UDPWiFi::stop() {
  _running = false;
  if (_thread.joinable()) _thread.join();

  if (_sockfd >= 0) {
    close(_sockfd);
    _sockfd = -1;
  }
}

float normalize(int val) {
  return std::roundf(((val - 2048.0f) / 2047.0f) * 100.0f) /
         100.0f;  // normalize and rounded to 2 decimal places
}

void UDPWiFi::listenLoop() {
  uint8_t buffer[5];

  while (_running) {
    ssize_t len = recv(_sockfd, buffer, sizeof(buffer), 0);
    if (len == 5) {  // change to 5 when button is implemented
      int16_t y = (buffer[0] << 8) | buffer[1];
      int16_t x = (buffer[2] << 8) | buffer[3];
      uint8_t btn = buffer[4];
      _x = x;
      _y = y;
      _btn = btn;
      // std::cout << "Received: X=" << _x << ", Y=" << _y << ", Btn=" << static_cast<int>(btn)
      //           << std::endl;
    }
    // Sleep briefly to avoid busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

float UDPWiFi::getX() const { return normalize(_x.load()); }
float UDPWiFi::getY() const { return normalize(_y.load()); }
uint8_t UDPWiFi::getBtn() const { return _btn.load(); }

}  // namespace comm
