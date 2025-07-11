#pragma once

#include "UdpBase.h"
#include <string>
#include <netinet/in.h>
#include <google/protobuf/message.h>

namespace comm {

class UdpSender : public UdpBase {
 public:
  UdpSender(const std::string& dest_addr, int port);

  // Send raw data
  bool sendData(const void* data, size_t size);

  // Send protobuf message
  bool sendMessage(const google::protobuf::Message& message);

 private:
  struct sockaddr_in dest_addr_in_;
  std::string buffer_;  // Reusable buffer for serialization
  bool initializeSocket();
};

}  // namespace comm