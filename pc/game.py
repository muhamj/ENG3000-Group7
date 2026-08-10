import os
import sys

import pygame

try:
    import serial_reader
except ImportError:
    serial_reader = None

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGES_DIR = os.path.join(BASE_DIR, 'images')
TITLE_IMAGE = os.path.join(IMAGES_DIR, 'title.jpg')
MOLE_IMAGE = os.path.join(IMAGES_DIR, 'mole.png')


def load_image(path):
    if not os.path.exists(path):
        print(f'Image not found: {path}')
        sys.exit(1)
    return pygame.image.load(path)


def scale_to_fit(surface, max_width, max_height):
    width, height = surface.get_size()
    scale = min(max_width / width, max_height / height, 1.0)
    if scale < 1.0:
        return pygame.transform.smoothscale(surface, (max(1, int(width * scale)), max(1, int(height * scale))))
    return surface


def main():
    pygame.init()
    pygame.display.set_caption('Simple Mole Game')

    width = 500
    height = 800
    screen = pygame.display.set_mode((width, height))

    title_surface = load_image(TITLE_IMAGE).convert_alpha()
    mole_surface = load_image(MOLE_IMAGE).convert_alpha()

    title_surface = scale_to_fit(title_surface, width - 20, 160)
    mole_surface = scale_to_fit(mole_surface, width - 20, 360)

    title_x = (width - title_surface.get_width()) // 2
    title_y = 20
    mole_x = (width - mole_surface.get_width()) // 2
    mole_y = height - mole_surface.get_height() - 20

    reader = None
    if serial_reader is not None:
        reader = serial_reader.open_serial_reader()
        if reader is not None:
            print(f'Using serial port: {reader.port}')

    distance_cm = None
    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if reader is not None:
            dist = reader.read_distance()
            if dist is not None:
                distance_cm = dist

        show_mole = distance_cm is None or distance_cm > 5.0

        screen.fill((255, 255, 255))
        screen.blit(title_surface, (title_x, title_y))
        if show_mole:
            screen.blit(mole_surface, (mole_x, mole_y))

        pygame.display.flip()
        clock.tick(60)

    if reader is not None:
        reader.close()
    pygame.quit()


if __name__ == '__main__':
    main()
