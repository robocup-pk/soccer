import serial
import struct

SERIAL_PORT = "/dev/ttyUSB0"   # Change this to your port, e.g., /dev/ttyAMA0, /dev/serial0
BAUDRATE = 115200

def main():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
        print(f"Listening on {SERIAL_PORT} @ {BAUDRATE} baud")

        while True:
            data = ser.read(17)
            if len(data) != 17:
                continue

            if data[0] != ord('x'):
                print("Invalid header:", data[0])
                continue

            # Unpack 4 little-endian int32 values
            values = struct.unpack('<4i', data[1:17])
            print(f"Received speeds: {values}")

if __name__ == "__main__":
    main()
