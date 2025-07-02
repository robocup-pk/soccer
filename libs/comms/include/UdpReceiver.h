#pragma once

#include "UdpBase.h"
#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <google/protobuf/message.h>

namespace comm {

class UdpReceiver : public UdpBase {
 public:
  UdpReceiver(const std::string& multicast_addr, int port);

  bool start();
  void stop();
  bool isRunning() const { return running_; }

  // Set callback for raw data
  void setRawCallback(std::function<void(const void*, size_t)> callback);

  // Set callback for protobuf messages (template method in implementation)
  template <typename MessageType>
  void setProtobufCallback(std::function<void(const MessageType&)> callback);

 protected:
  virtual bool processPacket(const void* data, size_t size);

 private:
  std::thread receive_thread_;
  std::atomic<bool> running_;

  // Callbacks
  std::mutex callback_mutex_;
  std::function<void(const void*, size_t)> raw_callback_;
  std::function<bool(const void*, size_t)> protobuf_parser_;

  bool initializeSocket();
  void receiveLoop();
};

// Template implementation for protobuf callback
template <typename MessageType>
void UdpReceiver::setProtobufCallback(std::function<void(const MessageType&)> callback) {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  protobuf_parser_ = [callback](const void* data, size_t size) -> bool {
    MessageType message;
    if (!message.ParseFromArray(data, size)) {
      std::cerr << "Failed to parse protobuf message" << std::endl;
      return false;
    }
    callback(message);
    return true;
  };
}

}  // namespace comm