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
TITLE_IMAGE = os.path.join(IMAGES_DIR, "title.png")
MOLE_IMAGE = os.path.join(IMAGES_DIR, "mole.png")
HAMMER_IMAGE = os.path.join(IMAGES_DIR, "hammer.png")
MOLE_DEAD_IMAGE = os.path.join(IMAGES_DIR, "mole dead.png")
HELP_BUTTON_IMAGE = os.path.join(IMAGES_DIR, "help button.png")
GO_BACK_BUTTON_IMAGE = os.path.join(IMAGES_DIR, "go back button.png")
EASY_IMAGE = os.path.join(IMAGES_DIR, "easy.png")
MEDIUM_IMAGE = os.path.join(IMAGES_DIR, "medium.png")
HARD_IMAGE = os.path.join(IMAGES_DIR, "hard.png")

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
HIT_DISTANCE_CM = 5.0
SENSOR_MAX_DISTANCE_CM = 500.0
SENSOR_TIMEOUT_MS = 500
# All four sensors are mounted along the top edge. The two sensors in each
# box are angled differently, so calibrate their horizontal zones separately.
SENSOR_COLUMN_MAP = {
    "s1a": 0,
    "s1b": 1,
    "s2a": 1,
    "s2b": 2,
}
# Approximate distance bands based on a 0.6 m sensor height and 1.4 m deep
# play area. Replace these with measured values after mounting the boxes.
DEPTH_ROW_THRESHOLDS_CM = (100.0, 160.0)
SCREEN_MARGIN = 20
MOLE_MAX_WIDTH = 120
MOLE_MAX_HEIGHT = 120
HAMMER_MAX_WIDTH = 80
HAMMER_MAX_HEIGHT = 80
GRID_SIZE = 3
MOLE_MIN_MS = 1000
MOLE_MAX_MS = 3000
MOLE_DEAD_DISPLAY_MS = 700
HAMMER_WIND_MS = 200
SCORE_FONT_SIZE = 72
# Game states
STATE_MAIN_MENU = "MAIN_MENU"
STATE_SELECT_DIFFICULTY = "SELECT_DIFFICULTY"
STATE_DEMO = "DEMO"
STATE_GAME = "GAME"

# Difficulty settings (min_ms, max_ms)
DIFFICULTY_SETTINGS = {
    "EASY": (1400, 3500),
    "MEDIUM": (1000, 3000),
    "HARD": (600, 1800),
}

DEFAULT_DIFFICULTY = "MEDIUM"


def list_serial_ports():
    if serial is None:
        return []
    return [port.device for port in serial.tools.list_ports.comports()]


def find_serial_port():
    ports = list_serial_ports()
    return ports[0] if ports else None


def parse_distance_line(line):
    """Parse the four-value frame emitted by the master ESP32."""
    text = line.strip().lower()
    if not text.startswith("distances:"):
        return None

    distances = {}
    for name, value_text in re.findall(
        r"(s[12][ab])\s*=\s*([-+]?(?:[0-9]*\.)?[0-9]+|nan)",
        text,
    ):
        try:
            value = float(value_text)
        except ValueError:
            continue
        if not math.isnan(value) and 0 < value <= SENSOR_MAX_DISTANCE_CM:
            distances[name] = value

    if len(distances) != 4:
        return None
    return distances


def nearest_sensor_cell(distances):
    """Map top-edge sensor readings to a coarse 3x3 grid cell."""
    nearest_name = min(distances, key=distances.get)
    nearest_distance = distances[nearest_name]
    row = sum(nearest_distance > threshold for threshold in DEPTH_ROW_THRESHOLDS_CM)
    column = SENSOR_COLUMN_MAP[nearest_name]
    return row * GRID_SIZE + column


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

    def read_distances(self):
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


def get_grid_area(title_surface):
    title_height = title_surface.get_height()
    title_y = SCREEN_MARGIN
    play_area_top = title_y + title_height + SCREEN_MARGIN
    # Compute cell size to fit horizontally and in the remaining vertical space
    grid_cell_size = min(
        (WINDOW_WIDTH - (SCREEN_MARGIN * 2)) / GRID_SIZE,
        (WINDOW_HEIGHT - play_area_top - SCREEN_MARGIN * 1) / GRID_SIZE,
    )

    grid_size_pixels = grid_cell_size * GRID_SIZE

    # Center horizontally
    grid_start_x = int((WINDOW_WIDTH - grid_size_pixels) // 2)

    # Center vertically within the play area below the title
    available_height = WINDOW_HEIGHT - play_area_top - SCREEN_MARGIN
    grid_start_y = int(play_area_top + max(0, (available_height - grid_size_pixels) // 2))

    return grid_start_x, grid_start_y, grid_cell_size


def random_grid_index(current_index=None):
    options = list(range(GRID_SIZE * GRID_SIZE))

    if current_index is not None:
        options.remove(current_index)

    return random.choice(options if current_index is not None else options)


def cell_to_position(cell_index, grid_start_x, grid_start_y, cell_size, mole_surface):
    row, col = divmod(cell_index, GRID_SIZE)
    x = grid_start_x + (col * cell_size) + (cell_size - mole_surface.get_width()) // 2
    y = grid_start_y + (row * cell_size) + (cell_size - mole_surface.get_height()) // 2
    return x, y


def draw_grid(screen, grid_start_x, grid_start_y, cell_size):
    # Draw a simple tic-tac-toe style grid: thick black interior lines
    grid_size_pixels = int(cell_size * GRID_SIZE)
    line_color = (0, 0, 0)
    # Medium thickness: scale modestly with cell size, keep between 4 and 8 px
    line_thickness = min(8, max(4, int(cell_size * 0.06)))

    # Draw interior horizontal lines
    for row in range(1, GRID_SIZE):
        y = int(grid_start_y + row * cell_size)
        pygame.draw.line(
            screen,
            line_color,
            (int(grid_start_x), y),
            (int(grid_start_x + grid_size_pixels), y),
            line_thickness,
        )

    # Draw interior vertical lines
    for col in range(1, GRID_SIZE):
        x = int(grid_start_x + col * cell_size)
        pygame.draw.line(
            screen,
            line_color,
            (x, int(grid_start_y)),
            (x, int(grid_start_y + grid_size_pixels)),
            line_thickness,
        )


# =============================================================================
# MENU RENDERING HELPERS
# =============================================================================

def draw_label(screen, label_font, grid_start_x, grid_start_y, cell_size, text, cell_index, colour=(0, 0, 0)):
    """Draw a menu label centred inside a grid cell."""
    col = cell_index % GRID_SIZE
    row = cell_index // GRID_SIZE
    cx = int(grid_start_x + col * cell_size + cell_size / 2)
    cy = int(grid_start_y + row * cell_size + cell_size / 2)
    surf = label_font.render(text, True, colour)
    screen.blit(surf, (cx - surf.get_width() // 2, cy - surf.get_height() // 2))


def draw_image_in_cell(screen, image, grid_start_x, grid_start_y, cell_size, cell_index):
    """Draw an image centred and contained within a grid cell."""
    col = cell_index % GRID_SIZE
    row = cell_index // GRID_SIZE
    cx = int(grid_start_x + col * cell_size + cell_size / 2)
    cy = int(grid_start_y + row * cell_size + cell_size / 2)
    screen.blit(image, (cx - image.get_width() // 2, cy - image.get_height() // 2))


# =============================================================================
# MAIN GAME LOOP
# =============================================================================
#
# The main loop is intentionally divided into clearly labelled SCREEN sections:
#   1. Main Menu
#   2. Difficulty Selection
#   3. Demo Screen
#   4. Game Screen
#
# Keeping each screen in its own section makes it much easier to find and edit
# the behaviour for a particular menu/screen.


def main():
    pygame.init()
    pygame.display.set_caption("3x3 Mole Grid")

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

    # Create a font and a title surface that will hold the score number
    font = pygame.font.SysFont(None, SCORE_FONT_SIZE, bold=True)
    title_height = SCORE_FONT_SIZE + 20
    title_surface = pygame.Surface((WINDOW_WIDTH, title_height), pygame.SRCALPHA)

    # Load mole alive and dead images
    mole_alive = load_image(MOLE_IMAGE).convert_alpha()
    mole_alive = scale_to_fit(mole_alive, MOLE_MAX_WIDTH, MOLE_MAX_HEIGHT)
    mole_dead = None
    if os.path.exists(MOLE_DEAD_IMAGE):
        try:
            mole_dead = load_image(MOLE_DEAD_IMAGE).convert_alpha()
            # Scale dead mole to fit target size and allow upscaling so it's clearly visible
            dw, dh = mole_dead.get_size()
            if dw > 0 and dh > 0:
                scale = min(MOLE_MAX_WIDTH / dw, MOLE_MAX_HEIGHT / dh)
                new_size = (max(1, int(dw * scale)), max(1, int(dh * scale)))
                mole_dead = pygame.transform.smoothscale(mole_dead, new_size)
        except SystemExit:
            mole_dead = None

    # Load hammer if available (follow mouse while inside window)
    hammer_surface = None
    if os.path.exists(HAMMER_IMAGE):
        try:
            hammer_surface = load_image(HAMMER_IMAGE).convert_alpha()
            hammer_surface = scale_to_fit(hammer_surface, HAMMER_MAX_WIDTH, HAMMER_MAX_HEIGHT)
        except SystemExit:
            hammer_surface = None

    title_x = 0
    title_y = SCREEN_MARGIN

    grid_start_x, grid_start_y, cell_size = get_grid_area(title_surface)
    menu_title_surface = load_image(TITLE_IMAGE).convert_alpha()
    menu_title_width = int(WINDOW_WIDTH * 1.2)
    menu_title_height = max(
        1,
        int(menu_title_surface.get_height() * menu_title_width / menu_title_surface.get_width()),
    )
    menu_title_surface = pygame.transform.smoothscale(
        menu_title_surface,
        (menu_title_width, menu_title_height),
    )
    button_image_size = max(1, int(cell_size * 0.8))
    help_button_surface = scale_to_fit(
        load_image(HELP_BUTTON_IMAGE).convert_alpha(),
        button_image_size,
        button_image_size,
    )
    go_back_button_surface = scale_to_fit(
        load_image(GO_BACK_BUTTON_IMAGE).convert_alpha(),
        button_image_size,
        button_image_size,
    )
    easy_surface = scale_to_fit(
        load_image(EASY_IMAGE).convert_alpha(),
        button_image_size,
        button_image_size,
    )
    medium_surface = scale_to_fit(
        load_image(MEDIUM_IMAGE).convert_alpha(),
        button_image_size,
        button_image_size,
    )
    hard_surface = scale_to_fit(
        load_image(HARD_IMAGE).convert_alpha(),
        button_image_size,
        button_image_size,
    )
    mole_cell_index = random.randint(0, GRID_SIZE * GRID_SIZE - 1)
    mole_state = "alive"  # 'alive' or 'dead'
    mole_dead_until = 0
    now = pygame.time.get_ticks()
    # apply default difficulty
    current_difficulty = DEFAULT_DIFFICULTY
    MOLE_MIN_CURRENT, MOLE_MAX_CURRENT = DIFFICULTY_SETTINGS[current_difficulty]
    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
    score = 0

    # UI fonts
    label_font = pygame.font.SysFont(None, 28, bold=True)

    # region SCREEN STATE 
    # ==========================================================================
    # SCREEN STATE
    # ==========================================================================
    state = STATE_MAIN_MENU
    prev_mouse_cell = None

    hammer_pressed = False
    hammer_press_start = 0
    HAMMER_PRESS_DURATION = 150  # milliseconds
    hammer_winding = False
    hammer_wind_start = 0

    reader = open_serial_reader()
    if reader is not None:
        print(f"Using serial port: {reader.port}")
    elif serial is None:
        print("pyserial not installed. Serial reading is disabled.")
    else:
        print("Serial port not found on COM4. Check the USB connection and COM port.")

    clock = pygame.time.Clock()
    running = True
    mouse_focused = False
    mouse_pos = (0, 0)
    sensor_cell = None
    sensor_last_update = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
        # Per-frame: update mouse proximity wind-up and mole timers
        try:
            mouse_focused = pygame.mouse.get_focused()
            mouse_pos = pygame.mouse.get_pos()
        except Exception:
            mouse_focused = False
            mouse_pos = (0, 0)

        # Convert the nearest sensor into the same cell input used by menus/game.
        if reader is not None:
            distances = reader.read_distances()
            if distances is not None:
                sensor_cell = nearest_sensor_cell(distances)
                sensor_last_update = pygame.time.get_ticks()
                print(
                    "Sensor position: "
                    f"cell={sensor_cell} nearest={min(distances, key=distances.get)}"
                )

        # Sensor input takes priority when a complete frame is available.
        mx, my = mouse_pos
        grid_size_pixels = int(cell_size * GRID_SIZE)
        if (
            mx >= int(grid_start_x)
            and mx < int(grid_start_x + grid_size_pixels)
            and my >= int(grid_start_y)
            and my < int(grid_start_y + grid_size_pixels)
        ):
            mcol = int((mx - grid_start_x) // cell_size)
            mrow = int((my - grid_start_y) // cell_size)
            mouse_cell = sensor_cell if sensor_cell is not None else mrow * GRID_SIZE + mcol
        else:
            mouse_cell = sensor_cell

        control_pos = mouse_pos
        if sensor_cell is not None:
            sensor_col = sensor_cell % GRID_SIZE
            sensor_row = sensor_cell // GRID_SIZE
            control_pos = (
                int(grid_start_x + (sensor_col + 0.5) * cell_size),
                int(grid_start_y + (sensor_row + 0.5) * cell_size),
            )

        now = pygame.time.get_ticks()
        if now - sensor_last_update > SENSOR_TIMEOUT_MS:
            sensor_cell = None

        # endregion


        # region SCREEN 1 - MAIN MENU
        # Handles navigation from the main menu only.
        # Start -> Difficulty Selection
        # Demo  -> Demo Screen
        if state == STATE_MAIN_MENU:
            # Navigate to Select Difficulty (square 4) or Demo (square 7) on enter
            if mouse_cell != prev_mouse_cell and mouse_cell is not None:
                if mouse_cell == 4:
                    # Play the hammer hit animation when selecting a menu option.
                    hammer_pressed = True
                    hammer_press_start = now
                    state = STATE_SELECT_DIFFICULTY
                    print("Entered Select Difficulty screen")
                elif mouse_cell == 7:
                    # Play the hammer hit animation when selecting a menu option.
                    hammer_pressed = True
                    hammer_press_start = now
                    state = STATE_DEMO
                    demo_active = True
                    demo_next_action = now + 600
                    # place demo mole
                    mole_cell_index = random.randint(0, GRID_SIZE * GRID_SIZE - 1)
                    mole_state = "alive"
                    mole_expire_time = now + 800
                    print("Entered Demo screen")


        # endregion

        # region SCREEN 2 - DIFFICULTY SELECTION
        # Handles choosing EASY, MEDIUM or HARD.
        if state == STATE_SELECT_DIFFICULTY:
            # Select difficulty on entering grid squares 1,3,5
            if mouse_cell != prev_mouse_cell and mouse_cell is not None:
                if mouse_cell == 7:
                    hammer_pressed = True
                    hammer_press_start = now
                    state = STATE_MAIN_MENU
                    print("Returned to main menu")
                elif mouse_cell == 1:
                    selected = "HARD"
                elif mouse_cell == 3:
                    selected = "EASY"
                elif mouse_cell == 5:
                    selected = "MEDIUM"
                else:
                    selected = None

                if selected is not None:
                    # Play the hammer hit animation when selecting a difficulty.
                    hammer_pressed = True
                    hammer_press_start = now
                    current_difficulty = selected
                    MOLE_MIN_CURRENT, MOLE_MAX_CURRENT = DIFFICULTY_SETTINGS[current_difficulty]
                    score = 0
                    mole_cell_index = random.randint(0, GRID_SIZE * GRID_SIZE - 1)
                    mole_state = "alive"
                    now = pygame.time.get_ticks()
                    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
                    state = STATE_GAME
                    print(f"Difficulty {current_difficulty} selected; starting game")


        # endregion

        # region SCREEN 3 - DEMO SCREEN
        
        # Will need to include demo on how it works

        if state == STATE_DEMO and mouse_cell != prev_mouse_cell and mouse_cell == 8:
            hammer_pressed = True
            hammer_press_start = now
            state = STATE_MAIN_MENU
            demo_active = False
            print("Returned to main menu")


        # endregion

        # region SCREEN 4 - GAME SCREEN
        # Handles the playable game: hammer movement, hits and scoring.
        if state == STATE_GAME:
            # Wind-up logic: start wind-up when mouse enters mole's cell
            if mouse_cell == mole_cell_index and mole_state == "alive" and not hammer_winding and not hammer_pressed:
                hammer_winding = True
                hammer_wind_start = now
            # cancel wind-up if mouse leaves before wind-up finishes
            if mouse_cell != mole_cell_index and hammer_winding:
                hammer_winding = False

            # If wind-up completed, trigger press and hit
            if hammer_winding and (now - hammer_wind_start) >= HAMMER_WIND_MS:
                hammer_winding = False
                hammer_pressed = True
                hammer_press_start = now
                # perform hit
                if mole_state == "alive":
                    mole_state = "dead"
                    score += 1
                    print(f"Hit! {score} points.")


        # endregion

        # region MOLE TIMERS
        # Automatic mole movement only applies to the playable game.
        # Handle mole timers: automatic movement and dead-display expiration
        # Only run automatic mole movement when in the actual GAME state.
        if state == STATE_GAME:
            if mole_state == "dead":
                if now >= mole_dead_until:
                    previous_index = mole_cell_index
                    mole_cell_index = random_grid_index(previous_index)
                    mole_state = "alive"
                    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
            elif mole_state == "alive":
                if now >= mole_expire_time:
                    previous_index = mole_cell_index
                    mole_cell_index = random_grid_index(previous_index)
                    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)


        # endregion

        # region RENDERING - COMMON BACKGROUND / SCORE / GRID
        screen.fill((255, 255, 255))
        # Score is only shown during the demo and actual gameplay.
        # It is hidden on the main menu and difficulty selection screens.
        if state in (STATE_DEMO, STATE_GAME):
            score_surf = font.render(str(score), True, (0, 0, 0))
            score_x = (WINDOW_WIDTH - score_surf.get_width()) // 2
            screen.blit(score_surf, (score_x, title_y))

        draw_grid(screen, grid_start_x, grid_start_y, cell_size)


        # endregion

        # region RENDERING - MENU SCREENS
        # Draw only the labels belonging to the current menu screen.

        # ---------------------------------------------------------------------
        # SCREEN 1 - MAIN MENU
        # ---------------------------------------------------------------------
        if state == STATE_MAIN_MENU:
            draw_image_in_cell(screen, mole_alive, grid_start_x, grid_start_y, cell_size, 4)
            draw_image_in_cell(screen, help_button_surface, grid_start_x, grid_start_y, cell_size, 7)
            title_overlap = int(menu_title_surface.get_height() * 0.23)
            title_draw_y = grid_start_y - title_overlap - 130
            title_draw_x = (WINDOW_WIDTH - menu_title_surface.get_width()) // 2
            screen.blit(menu_title_surface, (title_draw_x, title_draw_y))


        # ---------------------------------------------------------------------
        # SCREEN 2 - DIFFICULTY SELECTION
        # ---------------------------------------------------------------------
        if state == STATE_SELECT_DIFFICULTY:
            draw_image_in_cell(screen, hard_surface, grid_start_x, grid_start_y, cell_size, 1)
            draw_image_in_cell(screen, easy_surface, grid_start_x, grid_start_y, cell_size, 3)
            draw_image_in_cell(screen, medium_surface, grid_start_x, grid_start_y, cell_size, 5)
            draw_image_in_cell(screen, go_back_button_surface, grid_start_x, grid_start_y, cell_size, 7)


        # ---------------------------------------------------------------------
        # SCREEN 3 - DEMO SCREEN
        # ---------------------------------------------------------------------
        if state == STATE_DEMO:
            draw_image_in_cell(screen, go_back_button_surface, grid_start_x, grid_start_y, cell_size, 8)
            # show brief instruction lines near the top
            instr_font = pygame.font.SysFont(None, 22)
            lines = [
                "Demo: Move the mouse (no clicks required)",
                "Move the hammer into the mole's square to hit it",
                "Return: move cursor into bottom-right square",
            ]
            for i, line in enumerate(lines):
                s = instr_font.render(line, True, (0, 0, 0))
                screen.blit(s, (SCREEN_MARGIN, title_y + i * 22))


        # endregion

        # region RENDERING - GAME
        # Draw mole depending on state
        if state in (STATE_GAME):
            if mole_state == "alive":
                draw_surface = mole_alive
            else:
                draw_surface = mole_dead if mole_dead is not None else mole_alive

            if draw_surface is not None:
                mole_x, mole_y = cell_to_position(
                    mole_cell_index,
                    grid_start_x,
                    grid_start_y,
                    cell_size,
                    draw_surface,
                )
                screen.blit(draw_surface, (mole_x, mole_y))


        # endregion

        # region RENDERING - HAMMER CURSOR / ANIMATION
        # The hammer is shared by the playable game and demo.
        # Draw hammer cursor when mouse is focused in the window
        if hammer_surface is not None and (mouse_focused or sensor_cell is not None):
            pygame.mouse.set_visible(False)
            hx = int(control_pos[0] - hammer_surface.get_width() // 2)
            hy = int(control_pos[1] - hammer_surface.get_height() // 2)

            # Update press animation state by time
            if hammer_pressed:
                if (now - hammer_press_start) > HAMMER_PRESS_DURATION:
                    hammer_pressed = False

            if hammer_winding:
                # wind-up: rotate slightly up and offset upward
                wind_offset = max(4, hammer_surface.get_height() // 8)
                rotated = pygame.transform.rotate(hammer_surface, 15)
                rx = int(control_pos[0] - rotated.get_width() // 2)
                ry = int(control_pos[1] - rotated.get_height() // 2 - wind_offset)
                screen.blit(rotated, (rx, ry))
            elif hammer_pressed:
                # pressed: offset slightly downward and rotate for effect
                pressed_offset = max(6, hammer_surface.get_height() // 6)
                rotated = pygame.transform.rotate(hammer_surface, -20)
                rx = int(control_pos[0] - rotated.get_width() // 2)
                ry = int(control_pos[1] - rotated.get_height() // 2 + pressed_offset)
                screen.blit(rotated, (rx, ry))
            else:
                screen.blit(hammer_surface, (hx, hy))
        else:
            pygame.mouse.set_visible(True)
        # endregion

        # update previous mouse cell for edge-trigger detection
        prev_mouse_cell = mouse_cell

        pygame.display.flip()
        clock.tick(60)

    if reader is not None:
        reader.close()

    pygame.quit()


if __name__ == "__main__":
    main()
