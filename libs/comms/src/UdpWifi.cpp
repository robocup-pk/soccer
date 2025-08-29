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
    : _sockfd(-1),
      _port(port),
      _running(false),
      _y1(0.0f),
      _x1(0.0f),
      _y2(0.0f),
      _x2(0.0f),
      _btn(0) {}

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
  uint8_t buffer[17];

  while (_running) {
    ssize_t len = recv(_sockfd, buffer, sizeof(buffer), 0);
    if (len == 17) {
      float y1, x1, y2, x2;
      uint8_t btn;
      std::memcpy(&y1, buffer, 4);
      std::memcpy(&x1, buffer + 4, 4);
      std::memcpy(&y2, buffer + 8, 4);
      std::memcpy(&x2, buffer + 12, 4);
      btn = buffer[16];
      _y1 = y1;
      _x1 = x1;
      _y2 = y2;
      _x2 = x2;
      _btn = btn;
      // std::cout << "Received: Y1=" << y1 << ", X1=" << x1 << ", Y2=" << y2 << ", X2=" << x2 <<
      // ", Btn=" << static_cast<int>(btn) << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

float UDPWiFi::getY1() const { return _y1.load(); }
float UDPWiFi::getX1() const { return _x1.load(); }
float UDPWiFi::getY2() const { return _y2.load(); }
float UDPWiFi::getX2() const { return _x2.load(); }
uint8_t UDPWiFi::getBtn() const { return _btn.load(); }

}  // namespace comm