#ifndef SOCCER_LIBS_COMMS_INCLUDE_UDPBASE_H
#define SOCCER_LIBS_COMMS_INCLUDE_UDPBASE_H

#include <string>
#include <cstdint>
#include <iostream>

namespace comm {

class UdpBase {
 public:
  UdpBase(const std::string& address, int port);
  virtual ~UdpBase();

  bool isValid() const { return socket_fd_ >= 0; }
  int getSocketFd() const { return socket_fd_; }
  static bool isMulticastAddress(const std::string& address);

 protected:
  int socket_fd_;
  std::string address_;
  int port_;
};

}  // namespace comm
#endif  // SOCCER_LIBS_COMMS_INCLUDE_UDPBASE_H