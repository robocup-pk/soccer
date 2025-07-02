#include "VisionTypes.h"
#include "vision/ssl_vision_wrapper.pb.h"
#include <iostream>
#include <signal.h>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cmath>

bool running = true;

void handleSignal(int) { running = false; }

class VisionTracker {
 public:
  VisionTracker() : frame_count_(0), last_geometry_time_(0) {}

  void processDetection(const SSL_DetectionFrame& frame) {
    frame_count_++;

    std::cout << "\n=== Frame #" << frame.frame_number() << " ===\n";
    std::cout << "Camera ID: " << frame.camera_id() << "\n";
    std::cout << "Capture time: " << std::fixed << std::setprecision(3) << frame.t_capture()
              << "s\n";

    // Process balls
    if (frame.balls_size() > 0) {
      std::cout << "Balls detected: " << frame.balls_size() << "\n";
      for (int i = 0; i < frame.balls_size(); i++) {
        const auto& ball = frame.balls(i);
        std::cout << "  Ball " << i << ": pos=(" << std::setw(6) << std::setprecision(0)
                  << ball.x() << "," << std::setw(6) << std::setprecision(0) << ball.y()
                  << ") mm, "
                  << "confidence=" << std::setprecision(2) << ball.confidence();

        if (ball.has_z()) {
          std::cout << ", height=" << std::setprecision(1) << ball.z() << "mm";
        }
        std::cout << "\n";

        trackBallVelocity(ball, frame.t_capture());
      }
    } else {
      std::cout << "No balls detected\n";
    }

    // Process yellow robots
    if (frame.robots_yellow_size() > 0) {
      std::cout << "Yellow robots: " << frame.robots_yellow_size() << "\n";
      for (int i = 0; i < frame.robots_yellow_size(); i++) {
        const auto& robot = frame.robots_yellow(i);
        printRobotInfo("Y", robot);
      }
    }

    // Process blue robots
    if (frame.robots_blue_size() > 0) {
      std::cout << "Blue robots: " << frame.robots_blue_size() << "\n";
      for (int i = 0; i < frame.robots_blue_size(); i++) {
        const auto& robot = frame.robots_blue(i);
        printRobotInfo("B", robot);
      }
    }

    std::cout << "Total frames received: " << frame_count_ << std::endl;
  }

  void processGeometry(const SSL_GeometryData& geometry) {
    auto now = std::chrono::steady_clock::now();
    auto now_seconds = std::chrono::duration<double>(now.time_since_epoch()).count();

    // Only print geometry info occasionally to avoid spam
    if (now_seconds - last_geometry_time_ > 5.0) {
      std::cout << "\n=== Field Geometry ===\n";
      const auto& field = geometry.field();

      std::cout << "Field size: " << field.field_length() << "mm x " << field.field_width()
                << "mm\n";
      std::cout << "Goal size: " << field.goal_width() << "mm x " << field.goal_depth() << "mm";
      if (field.has_goal_height()) {
        std::cout << " x " << field.goal_height() << "mm";
      }
      std::cout << "\n";

      if (field.has_ball_radius()) {
        std::cout << "Ball radius: " << field.ball_radius() << "mm\n";
      }
      if (field.has_max_robot_radius()) {
        std::cout << "Max robot radius: " << field.max_robot_radius() << "mm\n";
      }

      std::cout << "Camera calibrations: " << geometry.calib_size() << "\n";

      last_geometry_time_ = now_seconds;
    }
  }

 private:
  void printRobotInfo(const std::string& team, const SSL_DetectionRobot& robot) {
    std::cout << "  " << team;
    if (robot.has_robot_id()) {
      std::cout << robot.robot_id();
    } else {
      std::cout << "?";
    }
    std::cout << ": pos=(" << std::setw(6) << std::setprecision(0) << robot.x() << ","
              << std::setw(6) << std::setprecision(0) << robot.y() << ") mm";

    if (robot.has_orientation()) {
      double angle_deg = robot.orientation() * 180.0 / M_PI;
      std::cout << ", angle=" << std::setw(5) << std::setprecision(1) << angle_deg << "°";
    }

    std::cout << ", conf=" << std::setprecision(2) << robot.confidence() << "\n";
  }

  void trackBallVelocity(const SSL_DetectionBall& ball, double timestamp) {
    if (have_previous_ball_) {
      double dt = timestamp - prev_ball_time_;
      if (dt > 0 && dt < 0.1) {  // Only calculate if reasonable time difference
        double dx = ball.x() - prev_ball_x_;
        double dy = ball.y() - prev_ball_y_;
        double vel_x = dx / dt;                                       // mm/s
        double vel_y = dy / dt;                                       // mm/s
        double speed = sqrt(vel_x * vel_x + vel_y * vel_y) / 1000.0;  // m/s

        if (speed > 0.1) {  // Only show if moving
          std::cout << "  Ball velocity: " << std::setprecision(2) << speed << " m/s\n";
        }
      }
    }

    prev_ball_x_ = ball.x();
    prev_ball_y_ = ball.y();
    prev_ball_time_ = timestamp;
    have_previous_ball_ = true;
  }

  int frame_count_;
  double last_geometry_time_;

  // Ball tracking
  bool have_previous_ball_ = false;
  double prev_ball_x_, prev_ball_y_, prev_ball_time_;
};

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  signal(SIGINT, handleSignal);

  std::cout << "=== SSL-Vision Data Receiver ===" << std::endl;
  std::cout << "This example receives and parses SSL-Vision protobuf data." << std::endl;

  VisionTracker tracker;

  // Create SSL-Vision receiver
  comm::SSLVisionReceiver receiver("224.5.23.2", 10006);

  // Set up detection callback
  receiver.setDetectionCallback(
      [&tracker](const SSL_DetectionFrame& frame) { tracker.processDetection(frame); });

  // Set up geometry callback
  receiver.setGeometryCallback(
      [&tracker](const SSL_GeometryData& geometry) { tracker.processGeometry(geometry); });

  if (!receiver.start()) {
    std::cerr << "Failed to start receiver. Make sure:" << std::endl;
    std::cerr << "1. You have permission to join the multicast group" << std::endl;
    std::cerr << "2. SSL-Vision data is being broadcast on 224.5.23.2:10006" << std::endl;
    std::cerr << "3. Try running the vision_sender_example in another terminal" << std::endl;
    return 1;
  }

  std::cout << "Receiving SSL-Vision data on 224.5.23.2:10006" << std::endl;
  std::cout << "Press Ctrl+C to exit..." << std::endl;

  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "\nShutting down..." << std::endl;
  receiver.stop();

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
