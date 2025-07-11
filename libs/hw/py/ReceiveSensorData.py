import serial
import struct
import time

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
HEADER_CHAR = b'x'
PACKET_SIZE = 21  # 1 byte header + 5 x int32_t

def parse_packet(packet):
    """
    Parse a 21-byte packet of format:
    b'x' + 4*int32 (rpm1-4) + 1*int32 (gyro)
    Returns: ([rpm1, rpm2, rpm3, rpm4], gyro)
    """
    if len(packet) != PACKET_SIZE:
        return None
    if packet[0:1] != HEADER_CHAR:
        return None

    try:
        # Unpack 5 int32_t starting from byte 1
        values = struct.unpack('<5i', packet[1:])  # little-endian, 5 integers
        return values[:4], values[4]
    except struct.error:
        return None

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} @ {BAUDRATE} baud")
            angle_deg = 0.0
            prev_time = time.time()

            while True:
                # Read until we get a valid packet
                sync_found = False
                while not sync_found:
                    byte = ser.read(1)
                    if byte == HEADER_CHAR:
                        rest = ser.read(20)  # Read remaining 20 bytes
                        packet = byte + rest
                        if len(packet) == PACKET_SIZE:
                            sync_found = True
                        else:
                            print("Incomplete packet, skipping")
                            continue

                result = parse_packet(packet)
                if result:
                    rpm_values, gyro_mdeg_per_s = result

                    current_time = time.time()
                    dt = current_time - prev_time
                    prev_time = current_time

                    angle_deg += (gyro_mdeg_per_s / 1000.0) * dt

                    print(f"RPMs: {rpm_values}, Gyro: {gyro_mdeg_per_s} m°/s | "
                          f"Δt: {dt:.3f}s | Angle: {angle_deg:.2f}°")
                else:
                    print("Parse failed")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()

