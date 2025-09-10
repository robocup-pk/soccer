#include "libserial/SerialPort.h"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef __APPLE__
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/IOBSD.h>
#endif

namespace LibSerial {

SerialPort::SerialPort() : fd_(-1), is_open_(false) {}

SerialPort::SerialPort(const std::string& port_name) : fd_(-1), is_open_(false) {
    Open(port_name);
}

SerialPort::~SerialPort() {
    if (is_open_) {
        Close();
    }
}

void SerialPort::Open(const std::string& port_name) {
    port_name_ = port_name;
    
    #ifdef __APPLE__
    // On macOS, try to open a serial port (this is mostly for MODEL mode anyway)
    fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ >= 0) {
        is_open_ = true;
        std::cout << "[macOS SerialPort] Opened: " << port_name << std::endl;
        
        // Configure basic serial settings
        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(fd_, TCSANOW, &options);
    } else {
        // For development/testing, we'll run in MODEL mode
        std::cout << "[macOS SerialPort] Could not open " << port_name << ", running in MODEL mode" << std::endl;
        is_open_ = true;  // Pretend we're open for MODEL mode compatibility
        fd_ = -1;
    }
    #else
    std::cout << "[macOS SerialPort] Non-macOS platform detected" << std::endl;
    is_open_ = true;
    fd_ = -1;
    #endif
}

void SerialPort::Close() {
    #ifdef __APPLE__
    if (fd_ >= 0) {
        close(fd_);
    }
    #endif
    fd_ = -1;
    is_open_ = false;
}

bool SerialPort::IsOpen() const {
    return is_open_;
}

void SerialPort::Write(const std::vector<uint8_t>& data) {
    if (!is_open_) return;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        write(fd_, data.data(), data.size());
    } else {
        // MODEL mode - just log what we would send
        std::cout << "[macOS SerialPort] MODEL: Would write " << data.size() << " bytes" << std::endl;
    }
    #endif
}

void SerialPort::Write(const std::string& data) {
    std::vector<uint8_t> buffer(data.begin(), data.end());
    Write(buffer);
}

void SerialPort::ReadByte(uint8_t& byte, int timeout_ms) {
    byte = 0;
    if (!is_open_) return;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        // Set timeout if specified
        if (timeout_ms > 0) {
            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(fd_, &read_fds);
            timeout.tv_sec = timeout_ms / 1000;
            timeout.tv_usec = (timeout_ms % 1000) * 1000;
            
            int result = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
            if (result <= 0) {
                return; // Timeout or error
            }
        }
        
        ssize_t bytes_read = read(fd_, &byte, 1);
        if (bytes_read <= 0) {
            byte = 0;
        }
    } else {
        // MODEL mode - return dummy data
        byte = 'x';  // Header byte for MODEL mode
    }
    #endif
}

size_t SerialPort::Read(std::vector<uint8_t>& buffer, size_t count, int timeout_ms) {
    if (!is_open_) return 0;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        buffer.resize(count);
        
        // Set timeout if specified
        if (timeout_ms > 0) {
            fd_set read_fds;
            struct timeval timeout;
            FD_ZERO(&read_fds);
            FD_SET(fd_, &read_fds);
            timeout.tv_sec = timeout_ms / 1000;
            timeout.tv_usec = (timeout_ms % 1000) * 1000;
            
            int result = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
            if (result <= 0) {
                return 0; // Timeout or error
            }
        }
        
        ssize_t bytes_read = read(fd_, buffer.data(), count);
        if (bytes_read > 0) {
            buffer.resize(bytes_read);
            return bytes_read;
        }
        return 0;
    } else {
        // MODEL mode - return dummy sensor data
        buffer.resize(count);
        // Fill with dummy data that looks like valid sensor readings
        std::fill(buffer.begin(), buffer.end(), 0);
        return count;
    }
    #else
    return 0;
    #endif
}

void SerialPort::FlushInputBuffer() {
    if (!is_open_) return;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        tcflush(fd_, TCIFLUSH);
    }
    #endif
}

void SerialPort::FlushOutputBuffer() {
    if (!is_open_) return;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        tcflush(fd_, TCOFLUSH);
    }
    #endif
}

void SerialPort::DrainWriteBuffer() {
    if (!is_open_) return;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        tcdrain(fd_);
    }
    #endif
}

size_t SerialPort::GetNumberOfBytesAvailable() {
    if (!is_open_) return 0;
    
    #ifdef __APPLE__
    if (fd_ >= 0) {
        int bytes_available = 0;
        ioctl(fd_, FIONREAD, &bytes_available);
        return bytes_available;
    } else {
        // MODEL mode - pretend we always have data available
        return 25;  // Enough for our sensor protocol
    }
    #else
    return 0;
    #endif
}

} // namespace LibSerial