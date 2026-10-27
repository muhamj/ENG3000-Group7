import json
import math
import os
import random
import sys
import urllib.error
import urllib.request

import pygame


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ASSETS_DIR = os.path.join(BASE_DIR, "assets")
IMAGES_DIR = ASSETS_DIR
FONTS_DIR = os.path.join(ASSETS_DIR, "fonts")
LOGO_IMAGE = os.path.join(IMAGES_DIR, "logo.png")
PIXEL_FONT_PATH = os.path.join(FONTS_DIR, "PressStart2P.ttf")
LED_FONT_PATH = os.path.join(FONTS_DIR, "VT323.ttf")

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
HIT_DISTANCE_CM = 5.0
# Sensors are reliable up to ~200cm; readings beyond this are treated as
# out of range rather than a genuine far reading, so the usable 0-200cm
# band gets the full left/right and near/far swing instead of being
# squeezed into a fraction of a much larger nominal range.
SENSOR_MAX_DISTANCE_CM = 200.0
SENSOR_TIMEOUT_MS = 500
BOX_BASELINE_CM = 150.0
POSITION_MAP_DEPTH_CM = 180.0
DEAD_ZONE_DEPTH_CM = 60.0
PLAYABLE_ROW_START_DEPTH_CM = 55.0
ROW_1_END_DEPTH_CM = 100.0
ROW_2_END_DEPTH_CM = 140.0
LEFT_COLUMN_END_CM = BOX_BASELINE_CM / 2.0 - 40.0
RIGHT_COLUMN_START_CM = BOX_BASELINE_CM / 2.0 + 40.0
POSITION_BOUNDARY_EPSILON_CM = 0.001

# game_wifi.py no longer binds its own UDP socket - sensor_monitor.py owns
# port 4210 and is the single process that reads the ESP32's UDP packets.
# This reads sensor_monitor.py's HTTP /data endpoint instead, so both
# programs can run at the same time without fighting over the same port.
# sensor_monitor.py must be running (with its HTTP dashboard enabled,
# i.e. not started with --no-http) for the game to get sensor data.
SENSOR_MONITOR_HOST = "localhost"
SENSOR_MONITOR_HTTP_PORT = 8000
SENSOR_MONITOR_DATA_URL = f"http://{SENSOR_MONITOR_HOST}:{SENSOR_MONITOR_HTTP_PORT}/data"
SENSOR_POLL_TIMEOUT_S = 0.2

# Both boxes (master + slave) each have one sensor aimed left and one
# aimed right. sensor1/sensor3 are the two boxes' left-facing sensors,
# sensor2/sensor4 are the two right-facing ones. Combining each side's
# pair gives redundant left/right coverage instead of relying on a
# single sensor per side.
LEFT_SENSOR_NAMES = ("sensor1", "sensor3")
RIGHT_SENSOR_NAMES = ("sensor2", "sensor4")
# Depth bands: near/mid/far rows. The far row starts at 100cm, so anyone
# standing 1m or further back already reads as the back row.
DEPTH_ROW_THRESHOLDS_CM = (50.0, 100.0)
SCREEN_MARGIN = 20
GRID_SIZE = 3
GRID_GAP = 10
MOLE_MIN_MS = 1000
MOLE_MAX_MS = 3000
MOLE_DEAD_DISPLAY_MS = 700
HAMMER_WIND_MS = 200
HAMMER_WIND_MS = 50
SCORE_FONT_SIZE = 72
# How long a player must continuously stay in one menu button's cell
# before it's confirmed and acted on. This applies only to the menu
# screens (main menu, difficulty select, demo) - the playable GAME
# state keeps its own fast HAMMER_WIND_MS hit timing, unchanged.
MENU_CONFIRM_MS = 2000
# Game states
STATE_MAIN_MENU = "MAIN_MENU"
STATE_GAME = "GAME"

# Difficulty settings (min_ms, max_ms)
DIFFICULTY_SETTINGS = {
    "EASY": (2500, 4500),
    "MEDIUM": (1800, 3500),
    "HARD": (1200, 2500),
}

DEFAULT_DIFFICULTY = "EASY"

# =============================================================================
# ARCADE CABINET COLOUR PALETTE
# =============================================================================
# Lifted from the arcade-cabinet UI redesign (whack_a_mole_ui_redesign.html):
# a dark cabinet body, Pac-Man-blue borders, amber/cream chase lights, and
# a hole-in-a-mound mole instead of a flat sprite.
COL_MAZE_BLUE = (33, 33, 222)
COL_MAZE_BLUE_LT = (77, 77, 255)
COL_BLUE_LT = (110, 150, 255)
COL_BLUE_DEEP = (40, 56, 210)
COL_CAB_BLACK = (0, 0, 0)
COL_WOOD = (8, 8, 12)
COL_BEZEL_HI = (138, 143, 148)
COL_BEZEL_MID = (85, 88, 92)
COL_BEZEL_LO = (48, 50, 54)
COL_SCREEN_BG = (0, 0, 0)
COL_MOUND = (92, 63, 39)
COL_MOUND_DK = (61, 41, 23)
COL_HOLE = (6, 4, 3)
COL_AMBER = (255, 255, 0)
COL_CREAM = (255, 224, 90)
COL_DOT = (255, 184, 151)
COL_MOLE_FUR = (168, 110, 58)
COL_MOLE_DARK = (10, 7, 4)
COL_MOLE_PAW = (168, 110, 58)


def extract_distances(packet):
    """Pull sensor1..sensor4 out of a packet dict (as returned by
    sensor_monitor.py's /data endpoint) into the same shape
    parse_udp_packet used to return: only valid, in-range readings,
    invalid/missing/out-of-range ones simply absent from the dict.

    sensor_monitor.py's own parser already converts negative readings to
    None rather than dropping them and doesn't apply a max-range cutoff,
    so that filtering is re-applied here to keep behaviour identical to
    the old direct-UDP path."""
    if not isinstance(packet, dict):
        return None

    distances = {}
    for name in ("sensor1", "sensor2", "sensor3", "sensor4"):
        value = packet.get(name)
        if value is None:
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if 0 < value <= SENSOR_MAX_DISTANCE_CM:
            distances[name] = value

    for name in ("x", "y"):
        try:
            value = float(packet[name])
        except (KeyError, TypeError, ValueError):
            continue
        if 0.0 <= value <= 1.0:
            distances[name] = value

    if not distances:
        return None
    return distances


def combine_left_right(distances):
    """Combine the two boxes' matching-side sensors into one left and one
    right reading, taking whichever of each side's two sensors is closer.
    Returns (left, right); either may be None if neither sensor on that
    side has a valid reading this frame."""
    left_candidates = [distances[name] for name in LEFT_SENSOR_NAMES if name in distances]
    right_candidates = [distances[name] for name in RIGHT_SENSOR_NAMES if name in distances]
    left = min(left_candidates) if left_candidates else None
    right = min(right_candidates) if right_candidates else None
    return left, right


def sensor_fraction_x(left, right):
    """Blend the combined left/right readings into a continuous 0..1
    horizontal position. 0.0 = far left, 1.0 = far right, 0.5 = centered."""
    if left is None and right is None:
        return None
    # Treat a missing reading as "far away" on that side so the position
    # still leans toward whichever side actually has a reading.
    left = SENSOR_MAX_DISTANCE_CM if left is None else left
    right = SENSOR_MAX_DISTANCE_CM if right is None else right

    closeness_left = max(0.0, SENSOR_MAX_DISTANCE_CM - left)
    closeness_right = max(0.0, SENSOR_MAX_DISTANCE_CM - right)
    total = closeness_left + closeness_right
    if total <= 0:
        return 0.5
    # Closer on the right pulls the fraction toward 1.0.
    return closeness_right / total


def sensor_fraction_row(left, right):
    """Pick a row using whichever side currently has the nearer reading."""
    candidates = [v for v in (left, right) if v is not None]
    if not candidates:
        return 0
    nearest_distance = min(candidates)
    return sum(nearest_distance > threshold for threshold in DEPTH_ROW_THRESHOLDS_CM)


def sensor_cell_from_fraction(fraction_x, row):
    """Convert a continuous 0..1 horizontal fraction and a row into a 3x3
    grid cell index, by splitting the width into three equal zones."""
    column = min(GRID_SIZE - 1, int(fraction_x * GRID_SIZE))
    return row * GRID_SIZE + column


def sensor_cell_from_position(x_fraction, y_fraction):
    """Map firmware coordinates to physical rows and 40 cm side columns."""
    x_cm = min(max(x_fraction, 0.0), 1.0) * BOX_BASELINE_CM
    depth_cm = (1.0 - y_fraction) * POSITION_MAP_DEPTH_CM
    if (
        depth_cm <= PLAYABLE_ROW_START_DEPTH_CM + POSITION_BOUNDARY_EPSILON_CM
        or depth_cm > POSITION_MAP_DEPTH_CM + POSITION_BOUNDARY_EPSILON_CM
    ):
        return None

    if x_cm <= LEFT_COLUMN_END_CM + POSITION_BOUNDARY_EPSILON_CM:
        column = 0
    elif x_cm >= RIGHT_COLUMN_START_CM - POSITION_BOUNDARY_EPSILON_CM:
        column = 2
    else:
        column = 1

    if depth_cm < ROW_1_END_DEPTH_CM - POSITION_BOUNDARY_EPSILON_CM:
        near_to_far_row = 0
    elif depth_cm < ROW_2_END_DEPTH_CM - POSITION_BOUNDARY_EPSILON_CM:
        near_to_far_row = 1
    else:
        near_to_far_row = 2
    row = GRID_SIZE - 1 - near_to_far_row
    return row * GRID_SIZE + column


class SensorMonitorReader:
    """Polls sensor_monitor.py's HTTP /data endpoint for the latest
    packet from each ESP32 board, instead of binding its own UDP socket.

    sensor_monitor.py is the single process that owns UDP port 4210 -
    this and sensor_monitor.py's own dashboard can now run at the same
    time without splitting incoming packets between two competing
    sockets bound to the same port."""

    def __init__(self, url=SENSOR_MONITOR_DATA_URL, timeout=SENSOR_POLL_TIMEOUT_S):
        self.url = url
        self.timeout = timeout
        self._last_seen_at = {}  # mac -> received_at, to detect new packets

    def open(self):
        # Nothing to bind up front; report whether sensor_monitor.py's
        # HTTP server is actually reachable right now so callers get the
        # same "could not open" signal open_udp_reader() used to give.
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout):
                return True
        except (urllib.error.URLError, OSError):
            return False

    def read_distances(self):
        """Return the distances dict for whichever board has the most
        recently updated packet, or None if nothing new is available.
        Mirrors UdpDistanceReader.read_distances()'s "None means no new
        frame this call" contract."""
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout) as response:
                packets = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, json.JSONDecodeError):
            return None

        if not isinstance(packets, dict) or not packets:
            return None

        newest_mac = None
        newest_packet = None
        newest_time = None
        for mac, packet in packets.items():
            received_at = packet.get("received_at")
            if received_at is None:
                continue
            if newest_time is None or received_at > newest_time:
                newest_time = received_at
                newest_mac = mac
                newest_packet = packet

        if newest_packet is None:
            return None

        # Only treat it as a fresh frame if this packet is newer than the
        # last one we already returned for this board - otherwise the
        # game would keep re-processing the same stale reading every
        # poll, which the old UDP path never did (each recvfrom() was a
        # genuinely new packet).
        if self._last_seen_at.get(newest_mac) == newest_time:
            return None
        self._last_seen_at[newest_mac] = newest_time

        return extract_distances(newest_packet)

    def close(self):
        pass


def open_sensor_reader(url=SENSOR_MONITOR_DATA_URL):
    reader = SensorMonitorReader(url=url)

    if reader.open():
        return reader
    return None


def load_image(path):
    if not os.path.exists(path):
        print(f"Image not found: {path}")
        sys.exit(1)
    return pygame.image.load(path)


def load_font(path, size):
    if os.path.exists(path):
        return pygame.font.Font(path, size)
    print(f"Font not found: {path}, falling back to default font")
    return pygame.font.SysFont(None, size, bold=True)


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


def get_grid_area(_title_surface=None):
    cabinet_width = min(520, int(WINDOW_WIDTH * 0.95))
    grid_width = cabinet_width - 74
    grid_cell_size = (grid_width - GRID_GAP * (GRID_SIZE - 1)) / GRID_SIZE
    grid_start_x = int((WINDOW_WIDTH - grid_width) // 2)
    grid_start_y = int((WINDOW_HEIGHT - grid_width) // 2)
    return grid_start_x, grid_start_y, grid_cell_size


def random_grid_index(current_index=None):
    options = list(range(GRID_SIZE * GRID_SIZE))

    if current_index is not None:
        options.remove(current_index)

    return random.choice(options if current_index is not None else options)


def cell_rect(cell_index, grid_start_x, grid_start_y, cell_size):
    row, col = divmod(cell_index, GRID_SIZE)
    step = cell_size + GRID_GAP
    x = grid_start_x + col * step
    y = grid_start_y + row * step
    return pygame.Rect(int(x), int(y), int(math.ceil(cell_size)), int(math.ceil(cell_size)))


def grid_cell_at_position(position, grid_start_x, grid_start_y, cell_size):
    relative_x = position[0] - grid_start_x
    relative_y = position[1] - grid_start_y
    step = cell_size + GRID_GAP
    col = int(relative_x // step)
    row = int(relative_y // step)
    if col < 0 or col >= GRID_SIZE or row < 0 or row >= GRID_SIZE:
        return None
    if relative_x - col * step >= cell_size or relative_y - row * step >= cell_size:
        return None
    return row * GRID_SIZE + col


def cell_center(cell_index, grid_start_x, grid_start_y, cell_size):
    rect = cell_rect(cell_index, grid_start_x, grid_start_y, cell_size)
    return rect.centerx, rect.centery


# =============================================================================
# CABINET / BEZEL RENDERING
# =============================================================================
# Draws the dark wood cabinet body, metal bezel and inset black screen that
# the whole game sits inside, matching the arcade cabinet mockup's frame.

def draw_cabinet_background(screen, field_rect):
    screen.fill((4, 4, 9))
    cabinet_rect = field_rect.inflate(46, 46)
    pygame.draw.rect(screen, (0, 0, 3), cabinet_rect.move(0, 8), border_radius=8)
    pygame.draw.rect(screen, COL_WOOD, cabinet_rect, border_radius=8)
    pygame.draw.rect(screen, (22, 22, 28), cabinet_rect, width=1, border_radius=8)

    bolt_positions = [
        (cabinet_rect.left + 14, cabinet_rect.top + 14),
        (cabinet_rect.right - 14, cabinet_rect.top + 14),
        (cabinet_rect.left + 14, cabinet_rect.bottom - 14),
        (cabinet_rect.right - 14, cabinet_rect.bottom - 14),
    ]
    for bx, by in bolt_positions:
        pygame.draw.circle(screen, COL_BEZEL_LO, (bx, by), 6)
        pygame.draw.circle(screen, COL_BEZEL_HI, (bx - 2, by - 2), 2)


def draw_bezel_and_field(screen, field_rect):
    bezel_rect = field_rect.inflate(18, 18)
    pygame.draw.rect(screen, COL_BEZEL_MID, bezel_rect, border_radius=10)
    pygame.draw.rect(screen, COL_BEZEL_HI, bezel_rect, width=2, border_radius=10)

    # Black screen field
    pygame.draw.rect(screen, COL_SCREEN_BG, field_rect, border_radius=4)
    pygame.draw.rect(screen, COL_CAB_BLACK, field_rect, width=2, border_radius=4)
    for scan_y in range(field_rect.top + 2, field_rect.bottom, 3):
        pygame.draw.line(screen, (8, 8, 12), (field_rect.left + 2, scan_y), (field_rect.right - 2, scan_y))


# =============================================================================
# CHASE LIGHT PERIMETER DOTS
# =============================================================================
# Mirrors buildPerimeterDots()/perimChase in the HTML mockup: dots walk the
# rectangular perimeter of a cell and pulse in sequence. `elapsed_ms` and
# `period_ms` drive the animation; `active` toggles whether dots pulse
# (title cell: always; difficulty cells: only while charging).

def perimeter_dot_positions(rect, count):
    w, h = rect.width, rect.height
    perimeter = 2 * (w + h)
    points = []
    for i in range(count):
        dist = (i / count) * perimeter
        if dist < w:
            x, y = dist, 0
        elif dist < w + h:
            x, y = w, dist - w
        elif dist < 2 * w + h:
            x, y = w - (dist - (w + h)), h
        else:
            x, y = 0, h - (dist - (2 * w + h))
        points.append((rect.x + x, rect.y + y))
    return points


def draw_perimeter_chase(screen, rect, count, elapsed_ms, period_ms, active):
    points = perimeter_dot_positions(rect, count)
    for i, (px, py) in enumerate(points):
        # Same phase-per-dot offset as the CSS animation-delay stagger.
        phase = ((elapsed_ms - (i * (period_ms / count))) % period_ms) / period_ms
        # perimChase keyframes: dim at 0%/100%, bright at 50%.
        brightness = math.sin(phase * math.pi)
        brightness = max(0.0, brightness)
        if active:
            glow = 0.12 + 0.88 * brightness
        else:
            glow = 0.1
        radius = 2 if glow < 0.5 else 3
        color = (
            int(COL_AMBER[0] * glow),
            int(COL_AMBER[1] * glow),
            int(COL_AMBER[2] * glow),
        )
        pygame.draw.circle(screen, color, (int(px), int(py)), radius)
        if active and glow > 0.7:
            glow_surf = pygame.Surface((14, 14), pygame.SRCALPHA)
            pygame.draw.circle(glow_surf, (*COL_AMBER, 60), (7, 7), 6)
            screen.blit(glow_surf, (int(px) - 7, int(py) - 7))


# =============================================================================
# MENU RENDERING HELPERS
# =============================================================================

def draw_panel_cell(screen, rect, border_color, border_width=3, radius=6):
    """Draw the dark rounded panel used for every menu/grid cell."""
    pygame.draw.rect(screen, (5, 5, 5), rect, border_radius=radius)
    pygame.draw.rect(screen, border_color, rect, width=border_width, border_radius=radius)


def draw_label(screen, label_font, rect, text, colour):
    """Draw a menu label centred inside a grid cell rect."""
    surf = label_font.render(text, True, colour)
    screen.blit(surf, (rect.centerx - surf.get_width() // 2, rect.centery - surf.get_height() // 2))


def draw_image_in_rect(screen, image, rect):
    """Draw an image centred and contained within a rect."""
    screen.blit(image, (rect.centerx - image.get_width() // 2, rect.centery - image.get_height() // 2))


def get_menu_hot_cells(state):
    """Return the difficulty buttons that accumulate menu dwell time."""
    if state == STATE_MAIN_MENU:
        return {3, 4, 5}
    return set()


def draw_dwell_progress_bar(screen, label_font, rect, progress):
    """Draw a horizontal progress bar near the bottom of a grid cell,
    filling left-to-right as `progress` (0.0-1.0) increases. Used as the
    visual countdown while a menu selection is being confirmed."""
    cell_size = rect.width
    bar_margin = max(6, int(cell_size * 0.08))
    bar_height = max(8, int(cell_size * 0.09))
    bar_x = rect.x + bar_margin
    bar_y = rect.bottom - bar_margin - bar_height
    bar_width = rect.width - 2 * bar_margin

    progress = max(0.0, min(1.0, progress))
    track_color = (40, 30, 10)
    fill_color = COL_AMBER
    border_color = COL_CREAM

    pygame.draw.rect(screen, track_color, (bar_x, bar_y, bar_width, bar_height))
    fill_width = int(bar_width * progress)
    if fill_width > 0:
        pygame.draw.rect(screen, fill_color, (bar_x, bar_y, fill_width, bar_height))
    pygame.draw.rect(screen, border_color, (bar_x, bar_y, bar_width, bar_height), 2)

    remaining_seconds = max(0.0, MENU_CONFIRM_MS * (1.0 - progress) / 1000.0)
    countdown = label_font.render(f"Confirming: {remaining_seconds:.1f}s", True, COL_CREAM)
    countdown_x = bar_x + (bar_width - countdown.get_width()) // 2
    countdown_y = bar_y - countdown.get_height() - 3
    screen.blit(countdown, (countdown_x, countdown_y))


def draw_help_tooltip(screen, anchor_rect, field_rect, title_font, body_font):
    tooltip_rect = pygame.Rect(
        anchor_rect.centerx - 170,
        anchor_rect.top - 114,
        340,
        106,
    )
    tooltip_rect.clamp_ip(field_rect)
    pygame.draw.rect(screen, (5, 5, 5), tooltip_rect, border_radius=6)
    pygame.draw.rect(screen, COL_AMBER, tooltip_rect, width=2, border_radius=6)
    title = title_font.render("HOW TO PLAY", True, COL_AMBER)
    screen.blit(title, (tooltip_rect.x + 12, tooltip_rect.y + 9))
    for index, text in enumerate((
        "Move to aim the hammer.",
        "Hover a difficulty for 2 seconds.",
        "Swing over a mole to score.",
    )):
        line = body_font.render(text, True, COL_CREAM)
        screen.blit(line, (tooltip_rect.x + 12, tooltip_rect.y + 28 + index * 22))


# =============================================================================
# MOLE / HOLE RENDERING (recreated from the SVG hole+mound+mole markup)
# =============================================================================
# Ports moleShapeMarkup() and the surrounding hole/mound SVG from the HTML
# mockup into pygame drawing calls. `pop_fraction` is 0.0 (fully down in
# the hole) to 1.0 (fully popped up), matching the CSS translateY() on
# .mole-inner. `hit` brightens the fur, matching .mole.hit's filter.

def draw_hole_and_mound(screen, rect):
    """Static hole/mound base for a cell, drawn every frame under the mole."""
    cx, cy = rect.centerx, rect.centery
    w, h = rect.width, rect.height

    mound_rect = pygame.Rect(0, 0, int(w * 0.82), int(h * 0.42))
    mound_rect.center = (cx, cy + int(h * 0.18))
    pygame.draw.rect(screen, COL_MOUND_DK, mound_rect, border_radius=int(mound_rect.height * 0.4))

    inner_mound = mound_rect.inflate(-int(w * 0.09), -int(h * 0.10))
    pygame.draw.rect(screen, COL_MOUND, inner_mound, border_radius=int(inner_mound.height * 0.44))

    hole_w, hole_h = int(w * 0.5), int(h * 0.19)
    pygame.draw.ellipse(screen, COL_HOLE, (cx - hole_w // 2, cy - hole_h // 2, hole_w, hole_h))
    pygame.draw.ellipse(screen, COL_MAZE_BLUE, (cx - hole_w // 2, cy - hole_h // 2, hole_w, hole_h), width=2)


def draw_mole(screen, rect, pop_fraction, hit=False):
    """Draw the cartoon mole (head, ears, eye-patches, eyes) clipped to the
    hole opening, offset vertically by pop_fraction like the CSS
    translateY() wind-up/pop-up/hit animation."""
    cx, cy = rect.centerx, rect.centery
    w, h = rect.width, rect.height

    hole_w, hole_h = int(w * 0.5), int(h * 0.19)
    hole_rect = pygame.Rect(cx - hole_w // 2, cy - hole_h // 2, hole_w, hole_h)

    # Mole body size, proportioned like the SVG viewBox (100x100, head ~56 wide)
    mole_size = int(min(w, h) * 0.62)
    mole_surf = pygame.Surface((mole_size, mole_size), pygame.SRCALPHA)
    ms = mole_size

    fur = COL_MOLE_FUR
    dark = COL_MOLE_DARK
    if hit:
        fur = tuple(min(255, int(c * 1.5)) for c in fur)

    # Head outline + fur (rounded top, flat-ish bottom like the SVG path)
    head_rect = pygame.Rect(int(ms * 0.20), int(ms * 0.24), int(ms * 0.60), int(ms * 0.64))
    pygame.draw.ellipse(mole_surf, dark, head_rect.inflate(6, 6))
    pygame.draw.ellipse(mole_surf, fur, head_rect)

    # Ears
    ear_r = int(ms * 0.075)
    pygame.draw.circle(mole_surf, dark, (int(ms * 0.32), int(ms * 0.33)), ear_r + 2)
    pygame.draw.circle(mole_surf, dark, (int(ms * 0.68), int(ms * 0.33)), ear_r + 2)
    pygame.draw.circle(mole_surf, fur, (int(ms * 0.32), int(ms * 0.33)), ear_r)
    pygame.draw.circle(mole_surf, fur, (int(ms * 0.68), int(ms * 0.33)), ear_r)

    # Eye patches (white) + pupils
    eye_r = int(ms * 0.075)
    left_eye = (int(ms * 0.39), int(ms * 0.47))
    right_eye = (int(ms * 0.61), int(ms * 0.47))
    pygame.draw.circle(mole_surf, (255, 255, 255), left_eye, eye_r)
    pygame.draw.circle(mole_surf, (255, 255, 255), right_eye, eye_r)
    pupil_r = int(ms * 0.036)
    pygame.draw.circle(mole_surf, dark, (left_eye[0] + 2, left_eye[1] + 2), pupil_r)
    pygame.draw.circle(mole_surf, dark, (right_eye[0] + 2, right_eye[1] + 2), pupil_r)

    if hit:
        # Small "starburst" cheeks for a whacked look
        pygame.draw.circle(mole_surf, (255, 255, 255, 90), (ms // 2, int(ms * 0.6)), int(ms * 0.05))

    # Vertical offset: pop_fraction 0 -> mole hidden below hole, 1 -> mole
    # popped fully up. Mirrors mole-inner's translateY(46px) -> translateY(-30px).
    down_offset = int(h * 0.46)
    up_offset = int(h * 0.30)
    offset = down_offset - (down_offset + up_offset) * pop_fraction

    mole_x = cx - ms // 2
    mole_y = cy - ms // 2 + offset

    # Clip to a window from the top of the mound down through the hole's
    # vertical center, so the mole appears to rise up out of the hole and
    # never spills out above/beside the mound. Wide enough to cover the
    # full mole width, not just the narrow hole opening.
    clip_top = rect.y + int(h * 0.05)
    clip_bottom = hole_rect.centery
    clip_rect = pygame.Rect(cx - ms // 2 - 2, clip_top, ms + 4, max(1, clip_bottom - clip_top))

    prev_clip = screen.get_clip()
    screen.set_clip(clip_rect)
    screen.blit(mole_surf, (mole_x, mole_y))
    screen.set_clip(prev_clip)

    # Redraw the hole itself on top so its far (upper) lip still reads in
    # front of the mole's body, matching the SVG's stacking order.
    pygame.draw.ellipse(screen, COL_HOLE, hole_rect)
    pygame.draw.ellipse(screen, COL_MAZE_BLUE, hole_rect, width=2)


def draw_hit_burst(screen, rect, label_font, alpha_fraction):
    """Floating '+1' text on a successful hit, matching .hit-burst/floatUp."""
    if alpha_fraction <= 0:
        return
    text_surf = label_font.render("+10", True, COL_AMBER)
    text_surf.set_alpha(int(255 * alpha_fraction))
    rise = int(26 * (1.0 - alpha_fraction))
    x = rect.centerx - text_surf.get_width() // 2
    y = rect.centery - text_surf.get_height() // 2 - rise
    screen.blit(text_surf, (x, y))


# =============================================================================
# HAMMER / RETICLE RENDERING
# =============================================================================
# Ports the reticle SVG (handle + head) and its wind-up/swing rotation from
# the mockup into a small pygame surface, redrawn each frame at the current
# rotation angle.

def build_hammer_surface(size, angle_degrees):
    """Draw the hammer at a given rotation (degrees, matching the CSS
    rotate() convention: negative = wound back, positive = swung down)."""
    base = pygame.Surface((size, size), pygame.SRCALPHA)
    s = size / 52.0  # SVG viewBox was 52x52

    def r(x, y, w, h):
        return pygame.Rect(int(x * s), int(y * s), max(1, int(w * s)), max(1, int(h * s)))

    # Handle
    pygame.draw.rect(base, (18, 12, 6), r(20, 14, 12, 30), border_radius=int(3 * s))
    pygame.draw.rect(base, (176, 122, 64), r(22, 16, 8, 26), border_radius=int(2 * s))
    pygame.draw.rect(base, (206, 152, 88), r(23, 18, 3, 16))
    # Head
    pygame.draw.rect(base, (18, 12, 6), r(6, 2, 40, 18), border_radius=int(5 * s))
    pygame.draw.rect(base, (182, 186, 192), r(9, 5, 34, 12), border_radius=int(4 * s))
    pygame.draw.rect(base, (214, 218, 223), r(12, 7, 14, 5), border_radius=int(2 * s))

    rotated = pygame.transform.rotate(base, -angle_degrees)
    return rotated


def hammer_angle_for_state(hammer_winding, wind_progress, hammer_pressed, press_progress):
    """Return the hammer's rotation angle in degrees for the current
    wind-up/press state, echoing the mockup's charging (-32 -> -95deg)
    and swing (-32 -> 34 -> -32deg) keyframes."""
    if hammer_winding:
        # Wind back further the longer the dwell/hover has been held.
        return -32 - (63 * max(0.0, min(1.0, wind_progress)))
    if hammer_pressed:
        # Swing down through the hit and settle back to neutral.
        p = max(0.0, min(1.0, press_progress))
        if p < 0.45:
            t = p / 0.45
            return -32 + t * (34 - (-32))
        t = (p - 0.45) / 0.55
        return 34 + t * (-32 - 34)
    return -32


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
    pygame.display.set_caption("Wack a Mole")

    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

    # Fonts: PressStart2P for pixel/arcade labels (buttons, HUD chrome),
    # VT323 for the LED-style score readout - matching 'PixelArcade' and
    # 'LEDMono' in the HTML mockup.
    pixel_font_title = load_font(PIXEL_FONT_PATH, 18)
    pixel_font_small = load_font(PIXEL_FONT_PATH, 9)
    led_font_score = load_font(LED_FONT_PATH, SCORE_FONT_SIZE)
    led_font_instructions = load_font(LED_FONT_PATH, 22)
    label_font = load_font(LED_FONT_PATH, 26)

    grid_start_x, grid_start_y, cell_size = get_grid_area()
    grid_extent = int(cell_size * GRID_SIZE + GRID_GAP * (GRID_SIZE - 1))
    field_rect = pygame.Rect(
        grid_start_x - 14,
        grid_start_y - 14,
        grid_extent + 28,
        grid_extent + 28,
    )
    cabinet_rect = field_rect.inflate(46, 46)
    # Logo (replaces the old title-number title bar on the main menu)
    logo_surface = load_image(LOGO_IMAGE).convert_alpha()
    logo_target_w = int(grid_extent * 0.8)
    logo_target_h = max(1, int(logo_surface.get_height() * logo_target_w / logo_surface.get_width()))
    logo_surface = pygame.transform.smoothscale(logo_surface, (logo_target_w, min(logo_target_h, int(cell_size * 0.9))))

    mole_cell_index = random.randint(0, GRID_SIZE * GRID_SIZE - 1)
    mole_state = "alive"  # 'alive' or 'dead'
    mole_dead_until = 0
    now = pygame.time.get_ticks()
    # apply default difficulty
    current_difficulty = DEFAULT_DIFFICULTY
    MOLE_MIN_CURRENT, MOLE_MAX_CURRENT = DIFFICULTY_SETTINGS[current_difficulty]
    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
    score = 0
    mole_pop_start = now
    mole_pop_from = "alive"
    hit_burst_until = 0

    # region SCREEN STATE 
    # ==========================================================================
    # SCREEN STATE
    # ==========================================================================
    state = STATE_MAIN_MENU
    # Dwell-to-confirm state for menu screens: which cell (if any) is
    # currently being held on, and when that hold began. A menu selection
    # only fires once the player has stayed on the same hot cell for
    # MENU_CONFIRM_MS continuously - see the MENU DWELL-TO-CONFIRM region
    # below. This does not apply to STATE_GAME.
    menu_dwell_cell = None
    menu_dwell_start = 0

    hammer_pressed = False
    hammer_press_start = 0
    HAMMER_PRESS_DURATION = 150  # milliseconds
    hammer_winding = False
    hammer_wind_start = 0

    reader = open_sensor_reader()
    if reader is not None:
        print(f"Reading sensor data from sensor_monitor.py at {SENSOR_MONITOR_DATA_URL}")
    else:
        print(
            f"Could not reach sensor_monitor.py at {SENSOR_MONITOR_DATA_URL}. "
            "Make sure sensor_monitor.py is running (with its HTTP dashboard enabled)."
        )

    clock = pygame.time.Clock()
    running = True
    mouse_focused = False
    mouse_pos = (0, 0)
    sensor_cell = None
    sensor_position = None
    sensor_last_update = 0
    back_to_menu_rect = pygame.Rect(field_rect.right - 68, field_rect.top + 6, 58, 22)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif (
                event.type == pygame.MOUSEBUTTONDOWN
                and event.button == 1
                and state == STATE_GAME
                and back_to_menu_rect.collidepoint(event.pos)
            ):
                state = STATE_MAIN_MENU
                hammer_winding = False
                menu_dwell_cell = None
            
        # Per-frame: update mouse proximity wind-up and mole timers
        try:
            mouse_focused = pygame.mouse.get_focused()
            mouse_pos = pygame.mouse.get_pos()
        except Exception:
            mouse_focused = False
            mouse_pos = (0, 0)

        # Combine both boxes' sensors into a cell index - both column
        # and row snap fully to the 3x3 grid, no smoothing.
        if reader is not None:
            distances = reader.read_distances()
            if distances is not None:
                if "x" in distances and "y" in distances:
                    sensor_cell = sensor_cell_from_position(distances["x"], distances["y"])
                    sensor_position = (distances["x"], distances["y"])
                    sensor_last_update = pygame.time.get_ticks()
                    print(f"Sensor position: cell={sensor_cell} (x={distances['x']:.3f}, y={distances['y']:.3f})")
                else:
                    sensor_position = None
                    left, right = combine_left_right(distances)
                    fraction_x = sensor_fraction_x(left, right)
                    if fraction_x is not None:
                        row = sensor_fraction_row(left, right)
                        sensor_cell = sensor_cell_from_fraction(fraction_x, row)
                        sensor_last_update = pygame.time.get_ticks()
                        print(f"Sensor position: cell={sensor_cell} (left={left}, right={right})")

        # Sensor input takes priority when a complete frame is available.
        mx, my = mouse_pos
        mouse_grid_cell = grid_cell_at_position(
            mouse_pos, grid_start_x, grid_start_y, cell_size
        )
        mouse_cell = sensor_cell if sensor_cell is not None else mouse_grid_cell

        control_pos = mouse_pos
        if sensor_position is not None:
            control_pos = (
                int(grid_start_x + sensor_position[0] * grid_extent),
                int(grid_start_y + sensor_position[1] * grid_extent),
            )
        elif sensor_cell is not None:
            snapped_rect = cell_rect(sensor_cell, grid_start_x, grid_start_y, cell_size)
            control_pos = snapped_rect.center

        now = pygame.time.get_ticks()
        if now - sensor_last_update > SENSOR_TIMEOUT_MS:
            sensor_cell = None
            sensor_position = None

        # endregion

        # region MENU DWELL-TO-CONFIRM
        # Menu screens (main menu, difficulty select, demo) require the
        # player to stay on a button's cell continuously for
        # MENU_CONFIRM_MS before it's confirmed, instead of firing the
        # instant the cell is entered. This avoids accidental selections
        # from someone just passing through a cell. STATE_GAME is
        # deliberately excluded - it keeps its own fast HAMMER_WIND_MS
        # hit timing further down, untouched by any of this.
        menu_hot_cells = get_menu_hot_cells(state)
        menu_confirmed_cell = None
        if menu_hot_cells and mouse_cell in menu_hot_cells:
            if mouse_cell != menu_dwell_cell:
                menu_dwell_cell = mouse_cell
                menu_dwell_start = now
            elif now - menu_dwell_start >= MENU_CONFIRM_MS:
                menu_confirmed_cell = mouse_cell
                # Reset immediately so a held cell doesn't re-fire every
                # subsequent frame; the resulting state/screen change
                # naturally starts a fresh dwell for whatever comes next.
                menu_dwell_cell = None
                menu_dwell_start = now
        else:
            menu_dwell_cell = None
            menu_dwell_start = now
        # endregion


        # region SCREEN 1 - MAIN MENU
        if state == STATE_MAIN_MENU:
            selected_difficulty = {3: "EASY", 4: "MEDIUM", 5: "HARD"}.get(menu_confirmed_cell)
            if selected_difficulty is not None:
                hammer_pressed = True
                hammer_press_start = now
                current_difficulty = selected_difficulty
                MOLE_MIN_CURRENT, MOLE_MAX_CURRENT = DIFFICULTY_SETTINGS[current_difficulty]
                score = 0
                mole_cell_index = random.randint(0, GRID_SIZE * GRID_SIZE - 1)
                mole_state = "alive"
                mole_pop_start = now
                mole_pop_from = "alive"
                mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
                state = STATE_GAME
                print(f"Difficulty {current_difficulty} selected; starting game")


        # endregion

        # region SCREEN 4 - GAME SCREEN
        # Handles the playable game: hammer movement, hits and scoring.
        # Deliberately NOT using the menu dwell-to-confirm logic above -
        # hits stay on the original fast HAMMER_WIND_MS timing.
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
                    mole_pop_start = now
                    mole_pop_from = "dead"
                    hit_burst_until = now + 550
                    mole_dead_until = now + 260
                    score += 10
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
                    mole_pop_start = now
                    mole_pop_from = "alive"
                    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)
            elif mole_state == "alive":
                if now >= mole_expire_time:
                    previous_index = mole_cell_index
                    mole_cell_index = random_grid_index(previous_index)
                    mole_pop_start = now
                    mole_pop_from = "alive"
                    mole_expire_time = now + random.randint(MOLE_MIN_CURRENT, MOLE_MAX_CURRENT)


        # endregion

        # region RENDERING - CABINET / COMMON BACKGROUND / SCORE / GRID
        draw_cabinet_background(screen, field_rect)
        draw_bezel_and_field(screen, field_rect)

        if state == STATE_GAME:
            score_surf = led_font_score.render(str(score), True, COL_CREAM)
            score_surf = pygame.transform.smoothscale(
                score_surf,
                (max(1, int(score_surf.get_width() * 0.42)), max(1, int(score_surf.get_height() * 0.42))),
            )
            screen.blit(score_surf, (cabinet_rect.centerx - score_surf.get_width() // 2, cabinet_rect.top + 3))

        # endregion

        # region RENDERING - MENU SCREENS
        # Draw only the labels belonging to the current menu screen.

        # ---------------------------------------------------------------------
        # SCREEN 1 - MAIN MENU
        # ---------------------------------------------------------------------
        if state == STATE_MAIN_MENU:
            title_rect = pygame.Rect(grid_start_x, grid_start_y, grid_extent, int(cell_size))
            draw_panel_cell(screen, title_rect, COL_MAZE_BLUE)
            draw_perimeter_chase(screen, title_rect, 28, now, 1800, True)
            draw_image_in_rect(screen, logo_surface, title_rect)

            diff_cells = {
                3: ("EASY", COL_BLUE_LT),
                4: ("MEDIUM", COL_AMBER),
                5: ("HARD", COL_BLUE_DEEP),
            }
            for idx, (label, color) in diff_cells.items():
                rect = cell_rect(idx, grid_start_x, grid_start_y, cell_size)
                charging = menu_dwell_cell == idx
                draw_panel_cell(screen, rect, color)
                draw_perimeter_chase(screen, rect, 20, now, 1800, charging)
                draw_label(screen, pixel_font_small, rect, label, color)

            for idx in (6, 8):
                blank_rect = cell_rect(idx, grid_start_x, grid_start_y, cell_size)
                draw_panel_cell(screen, blank_rect, COL_MAZE_BLUE)
                pygame.draw.circle(screen, COL_DOT, blank_rect.center, 3)

            help_rect = cell_rect(7, grid_start_x, grid_start_y, cell_size)
            help_color = COL_AMBER if mouse_cell == 7 else COL_MAZE_BLUE
            draw_panel_cell(screen, help_rect, help_color)
            draw_label(screen, pixel_font_title, help_rect, "?", COL_CREAM)
            help_label = pixel_font_small.render("HELP", True, COL_CREAM)
            screen.blit(help_label, (help_rect.centerx - help_label.get_width() // 2, help_rect.centery + 22))
            if mouse_cell == 7:
                draw_help_tooltip(screen, help_rect, field_rect, pixel_font_small, led_font_instructions)


        # endregion

        # region RENDERING - MENU DWELL PROGRESS BAR
        # Shows the 3-second confirmation countdown for whichever menu
        # button is currently being held on. Only drawn on menu screens -
        # menu_dwell_cell is always None while in STATE_GAME because the
        # dwell-to-confirm region above only tracks cells in
        # get_menu_hot_cells(state), which returns an empty set for GAME.
        if menu_dwell_cell is not None:
            dwell_progress = (now - menu_dwell_start) / MENU_CONFIRM_MS
            dwell_rect = cell_rect(menu_dwell_cell, grid_start_x, grid_start_y, cell_size)
            draw_dwell_progress_bar(
                screen,
                label_font,
                dwell_rect,
                dwell_progress,
            )
        # endregion

        # region RENDERING - GAME
        # Draw hole/mound + mole depending on state
        if state == STATE_GAME:
            draw_panel_cell(screen, back_to_menu_rect, COL_MAZE_BLUE, border_width=1, radius=4)
            draw_label(screen, pixel_font_small, back_to_menu_rect, "MENU", COL_CREAM)
            for idx in range(GRID_SIZE * GRID_SIZE):
                rect = cell_rect(idx, grid_start_x, grid_start_y, cell_size)
                draw_hole_and_mound(screen, rect)

            mole_rect = cell_rect(mole_cell_index, grid_start_x, grid_start_y, cell_size)
            pop_elapsed = now - mole_pop_start
            if mole_state == "alive":
                # Pop up quickly from wherever it started (mirrors the
                # mockup's fast .12s pop transition).
                pop_progress = min(1.0, pop_elapsed / 120.0)
                pop_fraction = pop_progress if mole_pop_from == "alive" else pop_progress
                draw_mole(screen, mole_rect, pop_fraction, hit=False)
            else:
                # Just hit: briefly show the "hit" brightened mole near the
                # top before it recedes back into the hole.
                if pop_elapsed < 100:
                    draw_mole(screen, mole_rect, 0.85, hit=True)
                else:
                    recede_progress = min(1.0, (pop_elapsed - 100) / 200.0)
                    draw_mole(screen, mole_rect, max(0.0, 0.85 * (1.0 - recede_progress)), hit=False)

            if now < hit_burst_until:
                alpha_fraction = (hit_burst_until - now) / 550.0
                draw_hit_burst(screen, mole_rect, pixel_font_small, alpha_fraction)


        # endregion

        # region RENDERING - HAMMER CURSOR / ANIMATION
        # The hammer is shared by the playable game and demo.
        # Draw hammer cursor only when we have live sensor input, so mouse
        # movement never drives it (this is a sensor-controlled cabinet).
        if sensor_cell is not None or mouse_focused:
            pygame.mouse.set_visible(False)

            wind_progress = 0.0
            if hammer_winding:
                wind_progress = min(1.0, (now - hammer_wind_start) / HAMMER_WIND_MS)

            press_progress = 0.0
            if hammer_pressed:
                if (now - hammer_press_start) > HAMMER_PRESS_DURATION:
                    hammer_pressed = False
                else:
                    press_progress = min(1.0, (now - hammer_press_start) / HAMMER_PRESS_DURATION)

            # In menu screens, "winding" is really the dwell-confirm countdown
            # on this cell - reuse that progress so the hammer visibly winds
            # back over the full MENU_CONFIRM_MS dwell, matching the mockup's
            # charging animation.
            menu_hot_cells_now = get_menu_hot_cells(state)
            if menu_hot_cells_now and menu_dwell_cell is not None:
                hammer_winding = True
                wind_progress = min(1.0, (now - menu_dwell_start) / MENU_CONFIRM_MS)
            elif state != STATE_GAME:
                hammer_winding = False

            angle = hammer_angle_for_state(hammer_winding, wind_progress, hammer_pressed, press_progress)
            hammer_size = max(36, int(cell_size * 0.5))
            hammer_surf = build_hammer_surface(hammer_size, angle)
            hx = control_pos[0] - hammer_surf.get_width() // 2
            hy = control_pos[1] - hammer_surf.get_height() // 2
            screen.blit(hammer_surf, (hx, hy))
        else:
            pygame.mouse.set_visible(True)
        # endregion

        pygame.display.flip()
        clock.tick(60)

    if reader is not None:
        reader.close()

    pygame.quit()


if __name__ == "__main__":
    main()