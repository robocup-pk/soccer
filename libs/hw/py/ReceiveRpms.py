import serial
import time
import re

# === Configuration ===
SERIAL_PORT = "/dev/ttyUSB0"   # Adjust as needed
BAUDRATE = 115200
HEADER_CHAR = 'x'

def extract_rpms(data_str):
    """
    Extracts 4 integer RPMs from a string like 'x123-45678+90-12'
    using a regular expression.
    """
    matches = re.findall(r'[-+]?\d+', data_str)
    if len(matches) == 4:
        return list(map(int, matches))
    return None

def main():
    try:
        with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
            print(f"Listening on {SERIAL_PORT} @ {BAUDRATE} baud")
            buffer = ""

            while True:
                incoming = ser.read().decode(errors='ignore')
                if not incoming:
                    continue

                buffer += incoming

                # Look for the header character 'x'
                if HEADER_CHAR in buffer:
                    # Keep only data starting from the latest 'x'
                    buffer = buffer[buffer.rfind(HEADER_CHAR):]

                    # Try to extract RPMs
                    rpms = extract_rpms(buffer)
                    if rpms:
                        print("Received RPMs:", rpms)
                        buffer = ""  # Clear buffer after successful parse

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")

if __name__ == "__main__":
    main()
