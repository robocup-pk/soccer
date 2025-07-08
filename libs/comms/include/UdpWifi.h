#pragma once
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>

namespace comm {

class UDPWiFi {
 public:
  UDPWiFi(uint16_t port = 4210);
  ~UDPWiFi();

  bool start();
  void stop();

  int16_t getX() const;
  int16_t getY() const;

 private:
  void listenLoop();

  int _sockfd;
  uint16_t _port;
  std::thread _thread;
  std::atomic<bool> _running;
  std::atomic<int16_t> _x;
  std::atomic<int16_t> _y;
};

}  // namespace comm
