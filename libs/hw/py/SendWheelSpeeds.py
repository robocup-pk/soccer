import serial
import struct
import time

# === Configuration ===
SERIAL_PORT = "/dev/ttyUSB0"     # Adjust if needed (e.g., /dev/ttyUSB1 or /dev/ttyACM0)
BAUDRATE = 115200

# === Wheel speeds in RPM ===
wheel_speeds = [0, 0, 0, 0]  # [FL, FR, BL, BR]

def main():
    try:
        # Open the serial port
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Opened {SERIAL_PORT} @ {BAUDRATE} baud")
            
            while True:
                # Pack data into the format: 1 byte header + 4 * int32
                packet = struct.pack('<c4i', b'x', *wheel_speeds)

                # Send the packet
                ser.write(packet)
                ser.flush()

                print("Sent:", wheel_speeds)
                time.sleep(0.5)  # Adjust as needed

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()
