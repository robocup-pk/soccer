import serial
import time

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200
HEADER_CHAR = 'x'

def parse_line(line):
    """
    Parses a line like: 'x0,0,0,0,-1616'
    Returns a tuple: ([rpm1, rpm2, rpm3, rpm4], gyro)
    """
    if not line.startswith(HEADER_CHAR):
        return None

    parts = line[1:].strip().split(',')  # skip 'x'
    if len(parts) != 5:
        return None

    try:
        values = list(map(int, parts))
        return values[:4], values[4]
    except ValueError:
        return None

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} @ {BAUDRATE} baud")
            angle_deg = 0.0
            prev_time = time.time()

            while True:
                line = ser.readline().decode(errors='ignore').strip()
                result = parse_line(line)

                if result:
                    rpm_values, gyro_mdeg_per_s = result

                    current_time = time.time()
                    dt = current_time - prev_time
                    prev_time = current_time

                    angle_deg += (gyro_mdeg_per_s / 1000.0) * dt

                    print(f"RPMs: {rpm_values}, Gyro: {gyro_mdeg_per_s} m°/s | "
                          f"Δt: {dt:.3f}s | Angle: {angle_deg:.2f}°")
                else:
                    print(f"Skipped: {line}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()
