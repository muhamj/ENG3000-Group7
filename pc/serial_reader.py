import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print('Please install pyserial: pip install pyserial')
    sys.exit(1)


def list_serial_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


def open_serial_port(port, baudrate):
    try:
        return serial.Serial(port, baudrate, timeout=1)
    except serial.SerialException as exc:
        print(f'Error opening {port}: {exc}')
        return None


def main():
    parser = argparse.ArgumentParser(description='Read sensor data from ESP32 over USB serial.')
    parser.add_argument('--port', help='Serial port', default=None)
    parser.add_argument('--baud', help='Baud rate', type=int, default=115200)
    args = parser.parse_args()

    port = args.port
    if port is None:
        ports = list_serial_ports()
        if not ports:
            print('No serial ports found. Connect the ESP32 and try again.')
            sys.exit(1)
        print('Available serial ports:')
        for p in ports:
            print('  ' + p)
        port = ports[0]
        print(f'Using first port: {port}')

    ser = open_serial_port(port, args.baud)
    if ser is None:
        sys.exit(1)

    print(f'Listening on {port} at {args.baud} baud...')
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            if not line:
                continue
            print(line)
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        ser.close()


if __name__ == '__main__':
    main()
