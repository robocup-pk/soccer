#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace LibSerial {

// Enums for compatibility with original libserial
enum class BaudRate {
    BAUD_115200 = 115200,
    BAUD_9600 = 9600,
    BAUD_57600 = 57600,
    BAUD_38400 = 38400
};

enum class CharacterSize {
    CHAR_SIZE_8 = 8,
    CHAR_SIZE_7 = 7,
    CHAR_SIZE_6 = 6,
    CHAR_SIZE_5 = 5
};

enum class Parity {
    PARITY_NONE = 0,
    PARITY_ODD = 1,
    PARITY_EVEN = 2
};

enum class StopBits {
    STOP_BITS_1 = 1,
    STOP_BITS_2 = 2
};

enum class FlowControl {
    FLOW_CONTROL_NONE = 0,
    FLOW_CONTROL_SOFTWARE = 1,
    FLOW_CONTROL_HARDWARE = 2
};

/**
 * @brief macOS-compatible SerialPort implementation
 */
class SerialPort {
public:
    SerialPort();
    explicit SerialPort(const std::string& port_name);
    ~SerialPort();

    // Basic operations
    void Open(const std::string& port_name);
    void Close();
    bool IsOpen() const;

    // I/O operations
    void Write(const std::vector<uint8_t>& data);
    void Write(const std::string& data);
    void ReadByte(uint8_t& byte, int timeout_ms = 0);
    size_t Read(std::vector<uint8_t>& buffer, size_t count, int timeout_ms = 0);
    
    // Buffer operations
    void FlushInputBuffer();
    void FlushOutputBuffer();
    void DrainWriteBuffer();
    size_t GetNumberOfBytesAvailable();

    // Configuration methods
    void SetBaudRate(BaudRate baud_rate) {}
    void SetCharacterSize(CharacterSize char_size) {}
    void SetParity(Parity parity) {}
    void SetStopBits(StopBits stop_bits) {}
    void SetFlowControl(FlowControl flow_control) {}

private:
    int fd_;
    bool is_open_;
    std::string port_name_;
};

} // namespace LibSerial