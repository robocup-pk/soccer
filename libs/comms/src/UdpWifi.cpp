#include "UdpWifi.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>  // Add this include
#include <iostream>

namespace comm {

UDPWiFi::UDPWiFi(uint16_t port) : _sockfd(-1), _port(port), _running(false), _x(0), _y(0) {}

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

void UDPWiFi::listenLoop() {
  uint8_t buffer[4];

  while (_running) {
    ssize_t len = recv(_sockfd, buffer, sizeof(buffer), 0);
    if (len == 4) {
      int16_t y = (buffer[0] << 8) | buffer[1];
      int16_t x = (buffer[2] << 8) | buffer[3];
      _x = x;
      _y = y;

      std::cout << "Received X: " << x << "  Y: " << y << std::endl;
    }
    // Sleep briefly to avoid busy-waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int16_t UDPWiFi::getX() const { return _x.load(); }
int16_t UDPWiFi::getY() const { return _y.load(); }

}  // namespace comm
