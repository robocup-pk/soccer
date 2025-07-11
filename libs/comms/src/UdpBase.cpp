#include "UdpBase.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace comm {

UdpBase::UdpBase(const std::string& address, int port)
    : socket_fd_(-1), address_(address), port_(port) {}

UdpBase::~UdpBase() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool UdpBase::isMulticastAddress(const std::string& address) {
  uint32_t addr_value = ntohl(inet_addr(address.c_str()));
  return (addr_value >= 0xE0000000 && addr_value <= 0xEFFFFFFF);
}

}  // namespace comm
