#ifndef SOCCER_LIBS_COMMS_INCLUDE_VISIONTYPES_H
#define SOCCER_LIBS_COMMS_INCLUDE_VISIONTYPES_H

#include "UdpSender.h"
#include "UdpReceiver.h"
#include <functional>

#include "vision/ssl_vision_wrapper.pb.h"
#include "vision/ssl_vision_detection.pb.h"
#include "vision/ssl_vision_geometry.pb.h"

namespace comm {

// Type aliases for SSL Vision
using VisionSender = UdpSender;

// Raw data receiver class for receiving any UDP data without protobuf parsing
class RawDataReceiver : public UdpReceiver {
 public:
  RawDataReceiver(const std::string& multicast_addr, int port)
      : UdpReceiver(multicast_addr, port) {
    // Set up raw callback as default
    setRawCallback([](const void*, size_t) {
      // Default: do nothing
    });
  }

  // This method is inherited from UdpReceiver, just making it explicit
  using UdpReceiver::setRawCallback;
};

class SSLVisionReceiver : public UdpReceiver {
 public:
  SSLVisionReceiver(const std::string& multicast_addr = "224.5.23.2", int port = 10006)
      : UdpReceiver(multicast_addr, port) {
    // Set up protobuf callback for SSL_WrapperPacket
    setProtobufCallback<SSL_WrapperPacket>([this](const SSL_WrapperPacket& packet) {
      packet_callback_(packet);

      if (packet.has_detection()) {
        detection_callback_(packet.detection());
      }

      if (packet.has_geometry()) {
        geometry_callback_(packet.geometry());
      }
    });
  }

  void setDetectionCallback(std::function<void(const SSL_DetectionFrame&)> callback) {
    detection_callback_ = callback;
  }

  void setGeometryCallback(std::function<void(const SSL_GeometryData&)> callback) {
    geometry_callback_ = callback;
  }

  void setPacketCallback(std::function<void(const SSL_WrapperPacket&)> callback) {
    packet_callback_ = callback;
  }

 private:
  std::function<void(const SSL_DetectionFrame&)> detection_callback_ =
      [](const SSL_DetectionFrame&) {};
  std::function<void(const SSL_GeometryData&)> geometry_callback_ = [](const SSL_GeometryData&) {};
  std::function<void(const SSL_WrapperPacket&)> packet_callback_ = [](const SSL_WrapperPacket&) {};
};
}  // namespace comm

#endif  // SOCCER_LIBS_COMMS_INCLUDE_VISIONTYPES_H