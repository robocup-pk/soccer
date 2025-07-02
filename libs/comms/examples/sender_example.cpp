#include "UdpSender.h"
#include "VisionTypes.h"
#include "vision/ssl_vision_wrapper.pb.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <signal.h>
#include <iomanip>

bool running = true;

void handleSignal(int) { running = false; }

class VisionSimulator {
 public:
  VisionSimulator() : frame_counter_(0) {}

  SSL_WrapperPacket createVisionFrame() {
    frame_counter_++;

    auto now = std::chrono::high_resolution_clock::now();
    double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();

    SSL_WrapperPacket packet;
    auto* detection = packet.mutable_detection();

    // Frame metadata
    detection->set_frame_number(frame_counter_);
    detection->set_t_capture(timestamp);
    detection->set_t_sent(timestamp + 0.001);
    detection->set_camera_id(0);

    // Create a ball moving in a circle
    double angle = frame_counter_ * 0.1;  // Slower movement for easier tracking
    double radius = 1500.0;               // 1.5 meters from center

    auto* ball = detection->add_balls();
    ball->set_confidence(0.95f);
    ball->set_x(static_cast<float>(radius * cos(angle)));
    ball->set_y(static_cast<float>(radius * sin(angle)));
    ball->set_z(0.0f);  // Ball on ground
    ball->set_pixel_x(320 + 100 * cos(angle));
    ball->set_pixel_y(240 + 100 * sin(angle));

    // Add yellow team robots (stationary for now)
    for (int id = 0; id < 3; ++id) {
      auto* robot = detection->add_robots_yellow();
      robot->set_confidence(0.9f);
      robot->set_robot_id(id);
      robot->set_x(-3000 + id * 1000);  // Spread along field
      robot->set_y(1000);               // Yellow side
      robot->set_orientation(0.0f);
      robot->set_pixel_x(200 + id * 100);
      robot->set_pixel_y(150);
      robot->set_height(150.0f);
    }

    // Add blue team robots
    for (int id = 0; id < 3; ++id) {
      auto* robot = detection->add_robots_blue();
      robot->set_confidence(0.9f);
      robot->set_robot_id(id);
      robot->set_x(-3000 + id * 1000);
      robot->set_y(-1000);               // Blue side
      robot->set_orientation(3.14159f);  // Facing opposite direction
      robot->set_pixel_x(200 + id * 100);
      robot->set_pixel_y(350);
      robot->set_height(150.0f);
    }

    // Send field geometry every 60 frames (once per second at 60 FPS)
    if (frame_counter_ % 60 == 1) {
      addFieldGeometry(packet);
    }

    return packet;
  }

 private:
  void addFieldGeometry(SSL_WrapperPacket& packet) {
    auto* geometry = packet.mutable_geometry();
    auto* field = geometry->mutable_field();

    // Standard SSL field dimensions (in mm)
    field->set_field_length(9000);                 // 9 meters
    field->set_field_width(6000);                  // 6 meters
    field->set_goal_width(1000);                   // 1 meter
    field->set_goal_depth(200);                    // 20 cm
    field->set_boundary_width(300);                // 30 cm
    field->set_penalty_area_depth(1000);           // 1 meter
    field->set_penalty_area_width(2000);           // 2 meters
    field->set_center_circle_radius(500);          // 50 cm
    field->set_line_thickness(10);                 // 1 cm
    field->set_goal_center_to_penalty_mark(1000);  // 1 meter
    field->set_goal_height(155);                   // 15.5 cm
    field->set_ball_radius(21.5f);                 // 2.15 cm
    field->set_max_robot_radius(90.0f);            // 9 cm
  }

  int frame_counter_;
};

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  signal(SIGINT, handleSignal);

  std::cout << "=== SSL-Vision Data Sender ===" << std::endl;
  std::cout << "This example sends simulated SSL-Vision protobuf data." << std::endl;

  // Create UDP sender for SSL-Vision
  comm::VisionSender sender("224.5.23.2", 10006);

  if (!sender.isValid()) {
    std::cerr << "Failed to create UDP sender. Check network permissions." << std::endl;
    return 1;
  }

  VisionSimulator simulator;
  int frames_sent = 0;
  auto start_time = std::chrono::steady_clock::now();

  std::cout << "Sending SSL-Vision data at 30 FPS to 224.5.23.2:10006" << std::endl;
  std::cout << "Data includes: moving ball, 3 yellow robots, 3 blue robots" << std::endl;
  std::cout << "Press Ctrl+C to stop..." << std::endl;

  while (running) {
    auto frame_start = std::chrono::steady_clock::now();

    // Create and send vision frame
    SSL_WrapperPacket packet = simulator.createVisionFrame();

    if (sender.sendMessage(packet)) {
      frames_sent++;

      // Print status every 30 frames (once per second)
      if (frames_sent % 30 == 0) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto seconds = std::chrono::duration<double>(elapsed).count();
        double fps = frames_sent / seconds;

        std::cout << "Frame #" << packet.detection().frame_number()
                  << " sent (avg FPS: " << std::fixed << std::setprecision(1) << fps << ")";

        if (packet.has_geometry()) {
          std::cout << " [with geometry]";
        }
        std::cout << std::endl;
      }
    } else {
      std::cerr << "Failed to send frame #" << packet.detection().frame_number() << std::endl;
    }

    // Maintain 30 FPS (33.33ms per frame)
    auto frame_duration = std::chrono::steady_clock::now() - frame_start;
    auto target_duration = std::chrono::milliseconds(33);

    if (frame_duration < target_duration) {
      std::this_thread::sleep_for(target_duration - frame_duration);
    }
  }

  std::cout << "\nSent " << frames_sent << " frames total." << std::endl;
  std::cout << "Sender stopped." << std::endl;

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
