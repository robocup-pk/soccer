#include "VisionTypes.h"
#include <iostream>

void print_detection(const SSL_DetectionFrame& frame) {
    // Ball info
    for (int i = 0; i < frame.balls_size(); ++i) {
        const auto& ball = frame.balls(i);
        std::cout << "Ball " << i << ": x=" << ball.x() << " y=" << ball.y();
        // Velocity fields are not in the proto by default, but if you add them, check with has_velocity_x()
        if (ball.HasExtension(velocity_x)) {
            std::cout << " vx=" << ball.GetExtension(velocity_x);
        }
        if (ball.HasExtension(velocity_y)) {
            std::cout << " vy=" << ball.GetExtension(velocity_y);
        }
        std::cout << std::endl;
    }

    // Yellow robots
    std::cout << "Yellow robots: " << frame.robots_yellow_size() << std::endl;
    for (int i = 0; i < frame.robots_yellow_size(); ++i) {
        const auto& robot = frame.robots_yellow(i);
        std::cout << "  Yellow " << i << ": id=" << robot.robot_id()
                  << " x=" << robot.x() << " y=" << robot.y();
        // If you have velocity fields, print them here
        std::cout << std::endl;
    }

    // Blue robots
    std::cout << "Blue robots: " << frame.robots_blue_size() << std::endl;
    for (int i = 0; i < frame.robots_blue_size(); ++i) {
        const auto& robot = frame.robots_blue(i);
        std::cout << "  Blue " << i << ": id=" << robot.robot_id()
                  << " x=" << robot.x() << " y=" << robot.y();
        // If you have velocity fields, print them here
        std::cout << std::endl;
    }
}

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    comm::SSLVisionReceiver receiver;

    receiver.setDetectionCallback([](const SSL_DetectionFrame& frame) {
        print_detection(frame);
    });

    receiver.start();

    std::cout << "Listening for SSL-Vision packets. Press Enter to quit." << std::endl;
    std::cin.get();

    receiver.stop();
    google::protobuf::ShutdownProtobufLibrary();
    return 0;
}