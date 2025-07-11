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

  float getX() const;
  float getY() const;
  uint8_t getBtn() const;

 private:
  void listenLoop();

  int _sockfd;
  uint16_t _port;
  std::thread _thread;
  std::atomic<bool> _running;
  std::atomic<int16_t> _x;
  std::atomic<int16_t> _y;
  std::atomic<uint8_t> _btn;
};

}  // namespace comm
