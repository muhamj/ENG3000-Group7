import argparse
import math
import sys

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


# Physical layout (metres) — must match mapping.ino
BOX_A_X = 0.00
BOX_B_X = 1.50
BASELINE = BOX_B_X - BOX_A_X

NUM_SENSORS = 4


def list_serial_ports():
    if serial is None:
        return []
    return [port.device for port in serial.tools.list_ports.comports()]


def find_serial_port():
    ports = list_serial_ports()
    return ports[0] if ports else None


def parse_reading_line(line):
    """
    Parse one CSV line from mapping.ino:
    seq,t_ms,d0_cm,d1_cm,d2_cm,d3_cm,dA_cm,dB_cm,x_m,y_m

    Returns a dict of the parsed fields, or None if the line isn't a
    valid data row (e.g. the header line or a garbled read).
    """
    text = line.strip()
    if not text or text.startswith('=') or text.startswith('seq'):
        return None

    parts = text.split(',')
    if len(parts) != 10:
        return None

    try:
        seq = int(parts[0])
        t_ms = int(parts[1])
        d_cm = [float(parts[2 + i]) for i in range(NUM_SENSORS)]
        dA_cm = float(parts[6])
        dB_cm = float(parts[7])
        x_raw, y_raw = parts[8], parts[9]
        x = None if x_raw == 'NaN' else float(x_raw)
        y = None if y_raw == 'NaN' else float(y_raw)
    except ValueError:
        return None

    return {
        'seq': seq,
        't_ms': t_ms,
        'd_cm': d_cm,       # [d0, d1, d2, d3]
        'dA_cm': dA_cm,
        'dB_cm': dB_cm,
        'x': x,
        'y': y,
    }


def pick_box_range(d_shallow, d_steep):
    """
    Given two ranges from the same box (shallow + steep heads),
    return the shorter valid one, or None if both invalid.
    Mirrors pickBoxRange() in mapping.ino.
    """
    valid_shallow = d_shallow > 0
    valid_steep = d_steep > 0

    if valid_shallow and valid_steep:
        return min(d_shallow, d_steep)
    elif valid_shallow:
        return d_shallow
    elif valid_steep:
        return d_steep
    else:
        return None


def trilaterate(dA_cm, dB_cm, baseline=BASELINE):
    """
    Trilaterate player (x, y) in metres from the two box ranges (cm).
    Box A at (0, 0), Box B at (baseline, 0). Mirrors the math in the
    main loop() of mapping.ino.

    Returns (x, y) or (None, None) if no valid solution.
    """
    if dA_cm is None or dB_cm is None or dA_cm <= 0 or dB_cm <= 0:
        return None, None

    d1 = dA_cm / 100.0  # m
    d2 = dB_cm / 100.0

    x = (baseline * baseline + d1 * d1 - d2 * d2) / (2.0 * baseline)
    y_squared = d1 * d1 - x * x

    if y_squared < 0:
        return None, None

    y = math.sqrt(y_squared)  # positive root = in front of the wall
    return x, y


def locate_player(distances_cm):
    """
    Full pipeline from raw 4-sensor readings (cm) to (x, y) position,
    matching the ESP32 firmware exactly:
      1. Pick shorter valid range per box.
      2. Trilaterate using both box ranges.
    """
    if len(distances_cm) != NUM_SENSORS:
        raise ValueError(f'Expected {NUM_SENSORS} sensor readings, got {len(distances_cm)}')

    dA_cm = pick_box_range(distances_cm[0], distances_cm[1])
    dB_cm = pick_box_range(distances_cm[2], distances_cm[3])

    if dA_cm is None or dB_cm is None:
        return None, None

    return trilaterate(dA_cm, dB_cm)


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

    def read_reading(self):
        """
        Read one line from the ESP32 and parse it into a reading dict
        (see parse_reading_line). Returns None for header/blank/garbled
        lines so callers can just loop and skip Nones.
        """
        if self.serial is None:
            return None
        try:
            line = self.serial.readline().decode('utf-8', errors='replace').strip()
            return parse_reading_line(line)
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
    parser = argparse.ArgumentParser(description='Read player position from ESP32 over USB serial.')
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
            reading = reader.read_reading()
            if reading is None:
                continue

            # Prefer the board's own x/y if present, otherwise recompute
            # locally from the raw per-sensor distances (useful if you
            # want to re-run the algorithm on logged raw data).
            x, y = reading['x'], reading['y']
            if x is None or y is None:
                x, y = locate_player(reading['d_cm'])

            if x is not None and y is not None:
                print(f'seq={reading["seq"]} x={x:.3f} y={y:.3f}')
            else:
                print(f'seq={reading["seq"]} no fix')
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        reader.close()


if __name__ == '__main__':
    main()