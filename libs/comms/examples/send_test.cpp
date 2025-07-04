#include "VisionTypes.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  comm::VisionSender sender("224.5.23.2", 10006);

  if (!sender.isValid()) {
    std::cerr << "Failed to create UDP sender." << std::endl;
    return 1;
  }

  int frame = 0;
  while (true) {
    SSL_WrapperPacket packet;
    auto* detection = packet.mutable_detection();

    // Frame metadata
    detection->set_frame_number(frame++);
    double timestamp =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    detection->set_t_capture(timestamp);
    detection->set_t_sent(timestamp + 0.001);
    detection->set_camera_id(0);

    // Ball moving in a circle
    double angle = frame * 0.05;
    double radius = 1000.0;
    auto* ball = detection->add_balls();
    ball->set_confidence(1.0f);
    ball->set_x(static_cast<float>(radius * std::cos(angle)));
    ball->set_y(static_cast<float>(radius * std::sin(angle)));
    ball->set_z(0.0f);
    ball->set_pixel_x(320 + 100 * std::cos(angle));
    ball->set_pixel_y(240 + 100 * std::sin(angle));

    // Robot moving in a circle (same as ball, but offset)
    auto* robot = detection->add_robots_yellow();
    robot->set_confidence(1.0f);
    robot->set_robot_id(0);
    robot->set_x(static_cast<float>(radius * std::cos(angle + M_PI / 2)));
    robot->set_y(static_cast<float>(radius * std::sin(angle + M_PI / 2)));
    robot->set_orientation(static_cast<float>(angle));
    robot->set_pixel_x(320 + 100 * std::cos(angle + M_PI / 2));
    robot->set_pixel_y(240 + 100 * std::sin(angle + M_PI / 2));
    robot->set_height(150.0f);

    sender.sendMessage(packet);

    std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30 FPS
  }

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}