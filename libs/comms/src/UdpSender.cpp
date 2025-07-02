#include "UdpSender.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>

namespace comm {

UdpSender::UdpSender(const std::string& dest_addr, int port) : UdpBase(dest_addr, port) {
  initializeSocket();
}

bool UdpSender::initializeSocket() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  std::memset(&dest_addr_in_, 0, sizeof(dest_addr_in_));
  dest_addr_in_.sin_family = AF_INET;
  dest_addr_in_.sin_addr.s_addr = inet_addr(address_.c_str());
  dest_addr_in_.sin_port = htons(port_);

  if (isMulticastAddress(address_)) {
    int ttl = 1;
    if (setsockopt(socket_fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0) {
      std::cerr << "Failed to set multicast TTL" << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    std::cout << "UDP multicast sender initialized for " << address_ << ":" << port_ << std::endl;
  } else {
    std::cout << "UDP unicast sender initialized for " << address_ << ":" << port_ << std::endl;
  }

  return true;
}

bool UdpSender::sendData(const void* data, size_t size) {
  if (socket_fd_ < 0) {
    return false;
  }

  int bytes_sent =
      sendto(socket_fd_, data, size, 0, (struct sockaddr*)&dest_addr_in_, sizeof(dest_addr_in_));

  if (bytes_sent < 0) {
    std::cerr << "Failed to send data: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool UdpSender::sendMessage(const google::protobuf::Message& message) {
  buffer_.clear();
  if (!message.SerializeToString(&buffer_)) {
    std::cerr << "Failed to serialize protobuf message" << std::endl;
    return false;
  }

  return sendData(buffer_.data(), buffer_.size());
}

}  // namespace comm
