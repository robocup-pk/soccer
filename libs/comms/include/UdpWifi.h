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

  float getX1() const;
  float getY1() const;
  float getX2() const;
  float getY2() const;
  uint8_t getBtn() const;

 private:
  void listenLoop();

  int _sockfd;
  uint16_t _port;
  std::thread _thread;
  std::atomic<bool> _running;
  std::atomic<float> _x1;
  std::atomic<float> _y1;
  std::atomic<float> _x2;
  std::atomic<float> _y2;
  std::atomic<uint8_t> _btn;
};

}