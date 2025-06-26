#include "MotorDriver.h"

namespace hw {

MotorDriver::MotorDriver(size_t numMotors) : num_motors_(numMotors), running_(false) {
  desired_speeds_.resize(num_motors_, 0);
  sensed_speeds_.resize(num_motors_, 0);  // Add this line to initialize sensed_speeds_
}

MotorDriver::~MotorDriver() { stop(); }

void MotorDriver::init(const std::string& port) {
  try {
    serial_port_.Open(port);
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  } catch (const LibSerial::OpenFailed&) {
    std::cerr << "Failed to open serial port: " << port << std::endl;
    throw;
  }
}

void MotorDriver::setSpeeds(const std::vector<int>& speeds) {
  if (speeds.size() != num_motors_) {
    std::cerr << "Speed vector size mismatch. Expected: " << num_motors_
              << ", Got: " << speeds.size() << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(speed_mutex_);
  desired_speeds_ = speeds;
}

void MotorDriver::start() {
  running_ = true;
  control_thread_ = std::thread(&MotorDriver::controlLoop, this);
  sense_thread_ = std::thread(&MotorDriver::senseLoop, this);
}

void MotorDriver::stop() {
  running_ = false;

  if (control_thread_.joinable()) {
    control_thread_.join();
  }

  if (sense_thread_.joinable()) {
    sense_thread_.join();
  }

  if (serial_port_.IsOpen()) {
    serial_port_.Close();
  }
}

void MotorDriver::controlLoop() {
  while (running_) {
    std::vector<int> current_speeds;
    {
      std::lock_guard<std::mutex> lock(speed_mutex_);
      current_speeds = desired_speeds_;
    }

    std::string command;
    // Send motor commands via serial
    for (size_t i = 0; i < num_motors_; ++i) {
      std::ostringstream oss;
      int speed = current_speeds[i];
      bool dir = (speed >= 0);
      speed = std::abs(speed);
      oss << std::setfill('0') << std::setw(3) << speed;
      command += oss.str();
    }
    command += "\n";  // Fix: Change "/n" to "\n"
    try {
      serial_port_.Write(command);
    } catch (const std::exception& e) {
      std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MotorDriver::senseLoop() {
  while (running_) {
    // Read sensor data from serial port
    try {
      if (serial_port_.IsDataAvailable()) {
        std::string data;
        serial_port_.ReadLine(data);

        // Remove any trailing newline/carriage return
        data.erase(data.find_last_not_of(" \n\r\t") + 1);

        // Parse the data string (assuming same format: continuous 3-digit speeds)
        if (data.length() == num_motors_ * 3) {
          std::vector<int> parsed_speeds;
          parsed_speeds.reserve(num_motors_);

          for (size_t i = 0; i < num_motors_; ++i) {
            size_t start_pos = i * 3;
            std::string speed_str = data.substr(start_pos, 3);

            try {
              int speed = std::stoi(speed_str);
              parsed_speeds.push_back(speed);
            } catch (const std::exception&) {
              // Invalid number, skip this reading
              parsed_speeds.clear();
              break;
            }
          }

          // If parsing was successful, update sensed_speeds_
          if (parsed_speeds.size() == num_motors_) {
            std::lock_guard<std::mutex> lock(speed_mutex_);
            sensed_speeds_ = parsed_speeds;
          }
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "Failed to read from serial port: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

}  // namespace hw