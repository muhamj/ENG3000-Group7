import argparse
import math
import re
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


def list_serial_ports():
    if serial is None:
        return []
    return [port.device for port in serial.tools.list_ports.comports()]


def find_serial_port():
    ports = list_serial_ports()
    return ports[0] if ports else None


def parse_distance_line(line):
    text = line.strip().lower()
    match = re.match(r'^distance\s*:\s*([-+]?[0-9]*\.?[0-9]+|nan)', text)
    if not match:
        return None
    try:
        value = float(match.group(1))
        return None if math.isnan(value) else value
    except ValueError:
        return None


class SerialDistanceReader:
    def __init__(self, port=None, baudrate=115200, timeout=0.1):
        self.serial = None
        self.port = port or find_serial_port()
        self.baudrate = baudrate
        self.timeout = timeout

    def open(self):
        if serial is None or self.port is None:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return True
        except serial.SerialException:
            self.serial = None
            return False

    def read_distance(self):
        if self.serial is None:
            return None
        try:
            line = self.serial.readline().decode('utf-8', errors='replace').strip()
            return parse_distance_line(line)
        except serial.SerialException:
            self.close()
            return None

    def close(self):
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None


def open_serial_reader(port=None, baudrate=115200, timeout=0.1):
    reader = SerialDistanceReader(port=port, baudrate=baudrate, timeout=timeout)
    if reader.open():
        return reader
    return None


def main():
    parser = argparse.ArgumentParser(description='Read sensor data from ESP32 over USB serial.')
    parser.add_argument('--port', help='Serial port', default=None)
    parser.add_argument('--baud', help='Baud rate', type=int, default=115200)
    args = parser.parse_args()

    reader = open_serial_reader(port=args.port, baudrate=args.baud)
    if reader is None:
        print('Failed to open serial port. Connect the ESP32 and try again.')
        sys.exit(1)

    print(f'Listening on {reader.port} at {reader.baudrate} baud...')
    try:
        while True:
            distance = reader.read_distance()
            if distance is not None:
                print(f'distance: {distance:.2f}')
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        reader.close()


if __name__ == '__main__':
    main()
