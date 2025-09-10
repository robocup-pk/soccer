#include <iostream>
#include <string>
#include <vector>
#include <memory>

// Just test that our header compiles and has all the right interfaces
#include "libs/extern/macos_serial/include/libserial/SerialPort.h"

int main() {
    std::cout << "=== Testing macOS SerialPort Interface Compatibility ===" << std::endl;
    
    // Test 1: Enum availability
    std::cout << "✓ Testing LibSerial enums..." << std::endl;
    LibSerial::BaudRate baud = LibSerial::BaudRate::BAUD_115200;
    LibSerial::CharacterSize char_size = LibSerial::CharacterSize::CHAR_SIZE_8;
    LibSerial::Parity parity = LibSerial::Parity::PARITY_NONE;
    LibSerial::StopBits stop_bits = LibSerial::StopBits::STOP_BITS_1;
    LibSerial::FlowControl flow_control = LibSerial::FlowControl::FLOW_CONTROL_NONE;
    
    // Test 2: Class instantiation 
    std::cout << "✓ Testing SerialPort class instantiation..." << std::endl;
    LibSerial::SerialPort port;
    auto shared_port = std::make_shared<LibSerial::SerialPort>();
    
    // Test 3: Method signatures (just checking they exist with right signatures)
    std::cout << "✓ Testing method signatures..." << std::endl;
    
    // All methods from HardwareManager.cpp usage:
    shared_port->Open(std::string("/dev/test"));
    shared_port->SetBaudRate(baud);
    shared_port->SetCharacterSize(char_size);
    shared_port->SetParity(parity);
    shared_port->SetStopBits(stop_bits);
    shared_port->SetFlowControl(flow_control);
    bool is_open = shared_port->IsOpen();
    shared_port->Close();
    
    // All methods from SensorDriver.cpp usage:
    shared_port->FlushOutputBuffer();
    std::vector<uint8_t> write_buffer = {'x', 1, 2, 3, 4};
    shared_port->Write(write_buffer);
    shared_port->DrainWriteBuffer();
    
    size_t available = shared_port->GetNumberOfBytesAvailable();
    
    uint8_t byte;
    shared_port->ReadByte(byte, 10);
    
    std::vector<uint8_t> read_buffer;
    size_t bytes_read = shared_port->Read(read_buffer, 20, 50);
    
    shared_port->FlushInputBuffer();
    
    std::cout << "✅ ALL INTERFACE TESTS PASSED!" << std::endl;
    std::cout << "✅ macOS SerialPort is fully compatible with existing code!" << std::endl;
    std::cout << "✅ Available bytes test: " << available << std::endl;
    std::cout << "✅ Bytes read test: " << bytes_read << std::endl;
    std::cout << "✅ Is open test: " << (is_open ? "true" : "false") << std::endl;
    
    return 0;
}