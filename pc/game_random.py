import math
import os
import random
import re
import sys

import pygame

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(BASE_DIR, "images")
TITLE_IMAGE = os.path.join(IMAGES_DIR, "title.jpg")
MOLE_IMAGE = os.path.join(IMAGES_DIR, "mole.png")

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
HIT_DISTANCE_CM = 5.0
SCREEN_MARGIN = 20
MOLE_MAX_WIDTH = 100
MOLE_MAX_HEIGHT = 100


def list_serial_ports():
    if serial is None:
        return []
    return [port.device for port in serial.tools.list_ports.comports()]


def find_serial_port():
    ports = list_serial_ports()
    return ports[0] if ports else None


def parse_distance_line(line):
    text = line.strip().lower()
    match = re.match(r"^distance\s*:\s*([-+]?[0-9]*\.?[0-9]+|nan)", text)
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
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
            )
            return True
        except serial.SerialException:
            self.serial = None
            return False

    def read_distance(self):
        if self.serial is None:
            return None

        try:
            line = self.serial.readline().decode(
                "utf-8",
                errors="replace",
            ).strip()

            if line:
                print(f"Serial raw: {line}")

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
    reader = SerialDistanceReader(
        port=port,
        baudrate=baudrate,
        timeout=timeout,
    )

    if reader.open():
        return reader
    return None


def load_image(path):
    if not os.path.exists(path):
        print(f"Image not found: {path}")
        sys.exit(1)
    return pygame.image.load(path)


def scale_to_fit(surface, max_width, max_height):
    width, height = surface.get_size()
    scale = min(max_width / width, max_height / height, 1.0)

    if scale < 1.0:
        new_size = (
            max(1, int(width * scale)),
            max(1, int(height * scale)),
        )
        return pygame.transform.smoothscale(surface, new_size)

    return surface


def random_mole_position(mole_surface, play_area_top):
    mole_width, mole_height = mole_surface.get_size()

    minimum_x = SCREEN_MARGIN
    maximum_x = max(
        minimum_x,
        WINDOW_WIDTH - mole_width - SCREEN_MARGIN,
    )

    minimum_y = play_area_top
    maximum_y = max(
        minimum_y,
        WINDOW_HEIGHT - mole_height - SCREEN_MARGIN,
    )

    return (
        random.randint(minimum_x, maximum_x),
        random.randint(minimum_y, maximum_y),
    )


def main():
    pygame.init()
    pygame.display.set_caption("Simple Mole Game")

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

    title_surface = load_image(TITLE_IMAGE).convert_alpha()
    mole_surface = load_image(MOLE_IMAGE).convert_alpha()

    title_surface = scale_to_fit(title_surface, WINDOW_WIDTH - 40, 160)
    mole_surface = scale_to_fit(
        mole_surface,
        MOLE_MAX_WIDTH,
        MOLE_MAX_HEIGHT,
    )

    title_x = (WINDOW_WIDTH - title_surface.get_width()) // 2
    title_y = SCREEN_MARGIN

    play_area_top = title_y + title_surface.get_height() + SCREEN_MARGIN
    mole_x, mole_y = random_mole_position(mole_surface, play_area_top)
    mole_visible = True

    reader = open_serial_reader()
    if reader is not None:
        print(f"Using serial port: {reader.port}")
    elif serial is None:
        print("pyserial not installed. Serial reading is disabled.")
    else:
        print("Serial port not found.")

    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                mole_x, mole_y = random_mole_position(
                    mole_surface,
                    play_area_top,
                )
                mole_visible = True
                print(f"Space pressed. Mole moved to: ({mole_x}, {mole_y})")

        if reader is not None:
            distance_cm = reader.read_distance()

            if distance_cm is not None:
                print(f"Received distance: {distance_cm:.2f} cm")

                if distance_cm <= HIT_DISTANCE_CM and mole_visible:
                    mole_visible = False
                    print("Mole hit!")

                elif distance_cm > HIT_DISTANCE_CM and not mole_visible:
                    mole_x, mole_y = random_mole_position(
                        mole_surface,
                        play_area_top,
                    )
                    mole_visible = True
                    print(f"Mole respawned at: ({mole_x}, {mole_y})")

        screen.fill((255, 255, 255))
        screen.blit(title_surface, (title_x, title_y))

        if mole_visible:
            screen.blit(mole_surface, (mole_x, mole_y))

        pygame.display.flip()
        clock.tick(60)

    if reader is not None:
        reader.close()

    pygame.quit()


if __name__ == "__main__":
    main()