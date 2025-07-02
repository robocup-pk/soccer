#include "UdpReceiver.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <errno.h>

namespace comm {

UdpReceiver::UdpReceiver(const std::string& multicast_addr, int port) 
    : UdpBase(multicast_addr, port), running_(false) {
  initializeSocket();
}

bool UdpReceiver::initializeSocket() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
    return false;
  }

  // Allow multiple sockets to use the same PORT number
  int yes = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
    std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Set up the local address structure
  struct sockaddr_in local_addr;
  std::memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(port_);

  // Bind to the multicast port
  if (bind(socket_fd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
    std::cerr << "Failed to bind socket to port " << port_ << ": " << strerror(errno) << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Join the multicast group if this is a multicast address
  if (isMulticastAddress(address_)) {
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(address_.c_str());
    mreq.imr_interface.s_addr = INADDR_ANY;

    if (setsockopt(socket_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      std::cerr << "Failed to join multicast group " << address_ << ": " << strerror(errno) << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    std::cout << "UDP multicast receiver initialized for " << address_ << ":" << port_ << std::endl;
  } else {
    std::cout << "UDP unicast receiver initialized for " << address_ << ":" << port_ << std::endl;
  }

  return true;
}

bool UdpReceiver::start() {
  if (!isValid()) {
    std::cerr << "Cannot start receiver: socket is not valid" << std::endl;
    return false;
  }

  if (running_.load()) {
    std::cerr << "Receiver is already running" << std::endl;
    return false;
  }

  running_.store(true);
  receive_thread_ = std::thread(&UdpReceiver::receiveLoop, this);
  return true;
}

void UdpReceiver::stop() {
  if (running_.load()) {
    running_.store(false);
    
    // Close socket to interrupt any blocking receive calls
    if (socket_fd_ >= 0) {
      shutdown(socket_fd_, SHUT_RDWR);
    }
    
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }
}

void UdpReceiver::setRawCallback(std::function<void(const void*, size_t)> callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  raw_callback_ = callback;
}

void UdpReceiver::receiveLoop() {
  const size_t BUFFER_SIZE = 65536; // 64KB buffer for UDP packets
  char buffer[BUFFER_SIZE];

  while (running_.load()) {
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    
    ssize_t bytes_received = recvfrom(socket_fd_, buffer, BUFFER_SIZE, 0,
                                      (struct sockaddr*)&sender_addr, &sender_len);
    
    if (bytes_received < 0) {
      if (running_.load()) {
        std::cerr << "Error receiving data: " << strerror(errno) << std::endl;
      }
      break;
    }
    
    if (bytes_received == 0) {
      continue; // No data received
    }

    // Process the received packet
    if (!processPacket(buffer, bytes_received)) {
      // Processing failed, but continue receiving
      continue;
    }
  }
}

bool UdpReceiver::processPacket(const void* data, size_t size) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  
  // Try protobuf parser first if available
  if (protobuf_parser_) {
    if (protobuf_parser_(data, size)) {
      return true;
    }
    // If protobuf parsing fails, fall through to raw callback
  }
  
  // Call raw callback if available
  if (raw_callback_) {
    raw_callback_(data, size);
    return true;
  }
  
  return false;
}

} // namespace comm
