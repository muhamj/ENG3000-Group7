#!/usr/bin/env python3
"""
Whack-a-Mole - arcade cabinet edition.
Run:
  pip install pygame
  python sensor_monitor.py        (in another terminal, for the sensors)
  python whack_a_mole_game.py     (option: --scale 1.3)

Keys:
  ESC  back to the menu (or quit, if already on the menu)
"""

import argparse
import json
import math
import os
import random
import sys
import urllib.error
import urllib.request

import pygame


# =============================================================================
# Game settings
# =============================================================================

UI_SCALE = 1.1           # overall window size; 1.0 = same size as the HTML mockup
FPS = 60

ROUND_MOLES = 30         # the round ends after this many moles have popped up
POINTS_PER_HIT = 10
RESULTS_MS = 5000        # how long the Game Over popup stays before the menu

# (new mole every N ms, each mole stays up for N ms)
DIFFICULTY_PRESETS = {
    "Easy":   (1500, 2000),
    "Medium": (1150, 1500),
    "Hard":   (900, 1100),
}


# =============================================================================
# Sensor settings (unchanged from the team's working game file)
# =============================================================================

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

# sensor_monitor.py owns UDP port 4210 and is the single process that reads
# the ESP32's UDP packets. The game reads sensor_monitor.py's HTTP /data
# endpoint instead, so both programs can run at the same time.
# sensor_monitor.py must be running (with its HTTP dashboard enabled,
# i.e. not started with --no-http) for the game to get sensor data.
SENSOR_MONITOR_HOST = "localhost"
SENSOR_MONITOR_HTTP_PORT = 8000
SENSOR_MONITOR_DATA_URL = f"http://{SENSOR_MONITOR_HOST}:{SENSOR_MONITOR_HTTP_PORT}/data"
SENSOR_POLL_TIMEOUT_S = 0.2

# Both boxes (master + slave) each have one sensor aimed left and one
# aimed right. sensor1/sensor3 are the two boxes' left-facing sensors,
# sensor2/sensor4 are the two right-facing ones.
LEFT_SENSOR_NAMES = ("sensor1", "sensor3")
RIGHT_SENSOR_NAMES = ("sensor2", "sensor4")
# Depth bands: near/mid/far rows. The far row starts at 100cm, so anyone
# standing 1m or further back already reads as the back row.
DEPTH_ROW_THRESHOLDS_CM = (50.0, 100.0)
GRID_SIZE = 3

# Timing for the hammer (same values as the team's file)
HAMMER_WIND_MS = 50      # stay on a mole's cell this long before the hit lands
MENU_CONFIRM_MS = 2000   # stay on Easy/Medium/Hard this long to select it


# =============================================================================
# Assets
# =============================================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def find_asset(name):
    for folder in (os.path.join(BASE_DIR, "assets"),
                   os.path.join(BASE_DIR, "assets", "fonts"),
                   BASE_DIR):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            return path
    return None


LOGO_PATH = find_asset("logo.png")
PIXEL_FONT_PATH = find_asset("PressStart2P.ttf")
LED_FONT_PATH = find_asset("VT323.ttf")


# =============================================================================
# Colours (the :root tokens from the HTML)
# =============================================================================

def hexc(h, a=255):
    h = h.lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), a)


MAZE_BLUE = hexc("#2121de")
MAZE_BLUE_LT = hexc("#4d4dff")
BLUE_LT = hexc("#6e96ff")
BLUE_DEEP = hexc("#2838d2")
CAB_WOOD = hexc("#08080c")
PANEL = hexc("#050505")
AMBER = hexc("#ffff00")
CREAM = hexc("#ffe05a")
RED_LED = hexc("#ff0000")
DOT = hexc("#ffb897")
WHITE = (255, 255, 255, 255)
BLACK = (0, 0, 0, 255)

DIFF_COLOURS = {"Easy": BLUE_LT, "Medium": AMBER, "Hard": BLUE_DEEP}
MENU_CELL_ACTIONS = {3: "Easy", 4: "Medium", 5: "Hard", 7: "help"}


# =============================================================================
# Sensor input (copied unchanged from the team's working game file)
# =============================================================================

def extract_distances(packet):
    """Pull sensor1..sensor4 (and x/y, if the firmware sends them) out of a
    packet dict as returned by sensor_monitor.py's /data endpoint. Only
    valid, in-range readings are kept."""
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
    right reading, taking whichever of each side's two sensors is closer."""
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
    left = SENSOR_MAX_DISTANCE_CM if left is None else left
    right = SENSOR_MAX_DISTANCE_CM if right is None else right

    closeness_left = max(0.0, SENSOR_MAX_DISTANCE_CM - left)
    closeness_right = max(0.0, SENSOR_MAX_DISTANCE_CM - right)
    total = closeness_left + closeness_right
    if total <= 0:
        return 0.5
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
    packet from each ESP32 board."""

    def __init__(self, url=SENSOR_MONITOR_DATA_URL, timeout=SENSOR_POLL_TIMEOUT_S):
        self.url = url
        self.timeout = timeout
        self._last_seen_at = {}  # mac -> received_at, to detect new packets

    def open(self):
        try:
            with urllib.request.urlopen(self.url, timeout=self.timeout):
                return True
        except (urllib.error.URLError, OSError):
            return False

    def read_distances(self):
        """Return the distances dict for whichever board has the most
        recently updated packet, or None if nothing new is available."""
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

        # Only a fresh frame if this packet is newer than the last one
        # already returned for this board.
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


# =============================================================================
# Small drawing helpers
# =============================================================================

def cubic_bezier(x1, y1, x2, y2):
    """CSS-style cubic-bezier easing function."""
    def bez(t, a, b):
        return 3 * a * t * (1 - t) ** 2 + 3 * b * t * t * (1 - t) + t ** 3

    def ease(x):
        if x <= 0:
            return 0.0
        if x >= 1:
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(24):
            mid = (lo + hi) / 2
            if bez(mid, x1, x2) < x:
                lo = mid
            else:
                hi = mid
        return bez((lo + hi) / 2, y1, y2)
    return ease


EASE_BACK = cubic_bezier(.34, 1.56, .64, 1)   # the mole "pop" overshoot
EASE_OUT = cubic_bezier(0, 0, .58, 1)


def lerp(a, b, t):
    return a + (b - a) * t


def lerp_colour(c1, c2, t):
    return tuple(int(lerp(a, b, t)) for a, b in zip(c1, c2))


def quad_points(p0, p1, p2, steps=18):
    pts = []
    for i in range(steps + 1):
        t = i / steps
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1]
        pts.append((x, y))
    return pts


def blur(surface, amount=4):
    """Cheap blur: shrink then grow."""
    w, h = surface.get_size()
    small = pygame.transform.smoothscale(surface, (max(1, w // amount), max(1, h // amount)))
    return pygame.transform.smoothscale(small, (w, h))


def rounded_mask_apply(surface, radius):
    mask = pygame.Surface(surface.get_size(), pygame.SRCALPHA)
    mask.fill((255, 255, 255, 0))
    pygame.draw.rect(mask, (255, 255, 255, 255), mask.get_rect(), border_radius=radius)
    surface.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    return surface


def wrap_text(font, text, width):
    words, lines, line = text.split(), [], ""
    for w in words:
        test = (line + " " + w).strip()
        if font.size(test)[0] <= width:
            line = test
        else:
            if line:
                lines.append(line)
            line = w
    if line:
        lines.append(line)
    return lines


# =============================================================================
# Layout (all sizes in "HTML pixels" multiplied by the UI scale)
# =============================================================================

class Layout:
    def __init__(self, s):
        self.s = s
        P = self.P
        margin_x, margin_top = P(28), P(22)
        cab_w = P(560)
        cab_pad, bez_pad, field_pad = P(14), P(9), P(14)
        hud_h, hud_gap = P(36), P(10)
        self.gap = P(10)

        cab_x, cab_y = margin_x, margin_top
        bez_x, bez_y = cab_x + cab_pad, cab_y + cab_pad
        bez_w = cab_w - 2 * cab_pad
        f_x, f_y = bez_x + bez_pad, bez_y + bez_pad
        f_w = bez_w - 2 * bez_pad
        inner_x = f_x + field_pad
        inner_w = f_w - 2 * field_pad

        self.hud = pygame.Rect(inner_x + P(2), f_y + field_pad, inner_w - P(4), hud_h)
        grid_y = self.hud.bottom + hud_gap
        self.grid = pygame.Rect(inner_x, grid_y, inner_w, inner_w)
        f_h = self.grid.bottom + field_pad - f_y
        self.field = pygame.Rect(f_x, f_y, f_w, f_h)
        self.bezel = pygame.Rect(bez_x, bez_y, bez_w, f_h + 2 * bez_pad)
        self.cabinet = pygame.Rect(cab_x, cab_y, cab_w, self.bezel.height + 2 * cab_pad)

        cell = (inner_w - 2 * self.gap) / 3
        self.cell_size = int(cell)
        self.cells = [pygame.Rect(int(inner_x + c * (cell + self.gap)),
                                  int(grid_y + r * (cell + self.gap)),
                                  int(cell), int(cell))
                      for r in range(3) for c in range(3)]
        self.title = self.cells[0].union(self.cells[2])

        self.W = cab_w + 2 * margin_x
        self.H = self.cabinet.bottom + P(26)

    def P(self, v):
        return int(round(v * self.s))


# =============================================================================
# The game
# =============================================================================

class Mole:
    DOWN, UP, HIT = 46, -30, -6      # translateY in the 100-unit cell box

    def __init__(self):
        self.up = False
        self.hit = False
        self.down_at = 0
        self.hit_at = 0
        self._anim = (self.DOWN, self.DOWN, 0)

    def move_to(self, target, now):
        self._anim = (self.offset(now), target, now)

    def offset(self, now):
        start, end, t0 = self._anim
        p = (now - t0) / 120.0
        return end if p >= 1 else lerp(start, end, EASE_BACK(p))

    def reset(self, now):
        self.up = self.hit = False
        self.move_to(self.DOWN, now)


class Game:
    def __init__(self, scale):
        pygame.init()
        pygame.display.set_caption("Whack-a-Mole")
        self.L = Layout(scale)
        self.screen = pygame.display.set_mode((self.L.W, self.L.H))
        self.clock = pygame.time.Clock()
        self._text_cache = {}
        self._font_cache = {}

        # Sensor reader - same as the team's file: connect once at startup.
        self.reader = open_sensor_reader()
        if self.reader is not None:
            print(f"Reading sensor data from sensor_monitor.py at {SENSOR_MONITOR_DATA_URL}")
        else:
            print(
                f"Could not reach sensor_monitor.py at {SENSOR_MONITOR_DATA_URL}. "
                "Make sure sensor_monitor.py is running (with its HTTP dashboard enabled)."
            )
        self.sensor_cell = None
        self.sensor_position = None
        self.sensor_last_update = 0

        self._build_static()

        # input
        self.cursor_pos = None
        self.cursor_cell = None
        self.wind_cell = None        # cell the hammer is winding up over
        self.wind_start = 0

        # menu
        self.state = "menu"
        self.charging = None
        self.charge_start = 0
        self.blocked_cell = None
        self.hammer_anim = None

        # round
        self.moles = [Mole() for _ in range(9)]
        self.bursts = {}
        self.difficulty = "Medium"
        self.score = 0
        self.moles_popped = 0
        self.moles_hit = 0
        self.score_bump_at = -10_000
        self.spawning = False
        self.next_spawn = 0
        self.end_at = None
        self.results_start = 0

    # ------------------------------------------------------------- fonts/text
    def font(self, kind, size):
        px = max(6, self.L.P(size))
        key = (kind, px)
        if key not in self._font_cache:
            path = PIXEL_FONT_PATH if kind == "pixel" else LED_FONT_PATH
            self._font_cache[key] = pygame.font.Font(path, px) if path else pygame.font.Font(None, int(px * 1.6))
        return self._font_cache[key]

    def text(self, kind, size, text, colour, glow=None):
        key = (kind, size, text, colour, glow)
        if key in self._text_cache:
            return self._text_cache[key]
        f = self.font(kind, size)
        surf = f.render(text, kind != "pixel", colour[:3])
        if glow:
            pad = self.L.P(8)
            out = pygame.Surface((surf.get_width() + 2 * pad, surf.get_height() + 2 * pad), pygame.SRCALPHA)
            g = f.render(text, True, glow[:3])
            g.set_alpha(glow[3])
            halo = pygame.Surface(out.get_size(), pygame.SRCALPHA)
            for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2), (0, 0)):
                halo.blit(g, (pad + dx, pad + dy))
            out.blit(blur(halo, 3), (0, 0))
            out.blit(blur(halo, 6), (0, 0))
            out.blit(surf, (pad, pad))
            surf = out
        self._text_cache[key] = surf
        return surf

    def blit_center(self, surf, center):
        self.screen.blit(surf, surf.get_rect(center=(int(center[0]), int(center[1]))))

    # ------------------------------------------------------------ static art
    def _build_static(self):
        L, P = self.L, self.L.P

        # page background: soft radial glow at the top
        bg = pygame.Surface((L.W, L.H))
        bg.fill(BLACK)
        far = math.hypot(L.W / 2, L.H)
        steps = 60
        for i in range(steps, 0, -1):
            r = far * 0.65 * i / steps
            c = lerp_colour(hexc("#0a0a12"), BLACK, i / steps)
            pygame.draw.circle(bg, c, (L.W // 2, 0), int(r))

        # cabinet with drop shadow
        shadow = pygame.Surface((L.cabinet.w + P(80), L.cabinet.h + P(110)), pygame.SRCALPHA)
        pygame.draw.rect(shadow, (0, 0, 0, 180), (P(40), P(60), L.cabinet.w, L.cabinet.h), border_radius=P(8))
        bg.blit(blur(shadow, 10), (L.cabinet.x - P(40), L.cabinet.y - P(30)))
        pygame.draw.rect(bg, CAB_WOOD, L.cabinet, border_radius=P(8))

        # metal bolts in the corners
        for bx, by in ((L.cabinet.left + P(14), L.cabinet.top + P(14)),
                       (L.cabinet.right - P(14), L.cabinet.top + P(14)),
                       (L.cabinet.left + P(14), L.cabinet.bottom - P(14)),
                       (L.cabinet.right - P(14), L.cabinet.bottom - P(14))):
            r = P(6)
            pygame.draw.circle(bg, hexc("#303236"), (bx, by), r)
            pygame.draw.circle(bg, hexc("#55585c"), (bx, by), int(r * .8))
            pygame.draw.circle(bg, hexc("#8a8f94"), (bx - r // 4, by - r // 4), int(r * .4))

        # brushed-metal bezel (diagonal gradient)
        bez = pygame.Surface(L.bezel.size, pygame.SRCALPHA)
        w, h = L.bezel.size
        stops = [(0, hexc("#8a8f94")), (.4, hexc("#55585c")), (1, hexc("#303236"))]
        for i in range(w + h):
            t = i / (w + h)
            for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
                if t0 <= t <= t1:
                    c = lerp_colour(c0, c1, (t - t0) / (t1 - t0))
                    break
            pygame.draw.line(bez, c, (i, 0), (i - h, h))
        bg.blit(rounded_mask_apply(bez, P(10)), L.bezel.topleft)

        pygame.draw.rect(bg, BLACK, L.field, border_radius=P(4))
        self.static_bg = bg

        # CRT overlay for the screen: scanlines + inner vignette
        ov = pygame.Surface(L.field.size, pygame.SRCALPHA)
        for y in range(0, L.field.h, 3):
            pygame.draw.line(ov, (0, 0, 0, 46), (0, y + 2), (L.field.w, y + 2))
        depth = P(48)
        for d in range(depth):
            a = int(191 * math.exp(-(d / P(18)) ** 2))
            if a:
                pygame.draw.rect(ov, (0, 0, 0, a), (d, d, L.field.w - 2 * d, L.field.h - 2 * d), 1)
        self.crt_overlay = ov

        # logo
        self.logo = None
        if LOGO_PATH:
            img = pygame.image.load(LOGO_PATH).convert_alpha()
            target_w = min(P(260), int(L.title.w * .8))
            target_h = int(img.get_height() * target_w / img.get_width())
            max_h = L.title.h - P(20)
            if target_h > max_h:
                target_w = int(target_w * max_h / target_h)
                target_h = max_h
            self.logo = pygame.transform.smoothscale(img, (target_w, target_h))

        # cell art (drawn big, then shrunk for smooth edges)
        cs = L.cell_size
        self.hole_art = self._render_units(cs, self._draw_hole)
        self.mole_art = self._render_units(cs, lambda s, k: self._draw_mole(s, k, False))
        self.mole_hit_art = self._render_units(cs, lambda s, k: self._draw_mole(s, k, True))
        self.hammer_art = self._render_hammer()

        # glow frames for the menu boxes
        self.glow_charge = self._box_glow(L.cells[3].size, (255, 255, 0), 90, P(16))
        self.glow_help = self._box_glow(L.cells[7].size, (255, 255, 0), 75, P(14))

        # chase-light dot sprites
        self.dot_core = pygame.Surface((P(5) + 1, P(5) + 1), pygame.SRCALPHA)
        pygame.draw.circle(self.dot_core, AMBER, (self.dot_core.get_width() // 2,) * 2, max(2, P(5) // 2))
        halo_r = P(9)
        halo = pygame.Surface((halo_r * 2, halo_r * 2), pygame.SRCALPHA)
        pygame.draw.circle(halo, (255, 255, 0, 200), (halo_r, halo_r), P(5))
        self.dot_halo = blur(halo, 3)
        self.title_dots = self._perimeter(L.title, 28)
        self.diff_dots = {i: self._perimeter(L.cells[i], 20) for i in (3, 4, 5)}

    def _render_units(self, size, draw_fn):
        k = size * 3 / 100.0
        big = pygame.Surface((size * 3, size * 3), pygame.SRCALPHA)
        draw_fn(big, k)
        return pygame.transform.smoothscale(big, (size, size))

    @staticmethod
    def _draw_hole(s, k):
        R = lambda x, y, w, h: pygame.Rect(int(x * k), int(y * k), int(w * k), int(h * k))
        pygame.draw.rect(s, hexc("#3d2917"), R(5, 46, 90, 48), border_radius=int(20 * k))
        pygame.draw.rect(s, hexc("#5c3f27"), R(10, 48, 80, 36), border_radius=int(16 * k))
        pygame.draw.ellipse(s, hexc("#060403"), R(23, 39, 54, 22))
        pygame.draw.ellipse(s, MAZE_BLUE, R(23, 39, 54, 22), max(1, int(1.2 * k)))

    @staticmethod
    def _draw_mole(s, k, hit):
        def col(h):
            c = hexc(h)
            if hit:
                c = tuple(min(255, int(v * 1.6)) for v in c[:3]) + (255,)
            return c

        def poly(points, colour):
            pygame.draw.polygon(s, colour, [(x * k, y * k) for x, y in points])

        outline = quad_points((22, 78), (22, 34), (50, 28)) + quad_points((50, 28), (78, 34), (78, 78))
        body = quad_points((25, 76), (25, 37), (50, 32)) + quad_points((50, 32), (75, 37), (75, 76))
        poly(outline, col("#0a0704"))
        poly(body, col("#a86e3a"))
        for cx in (34, 66):
            pygame.draw.circle(s, col("#0a0704"), (cx * k, 38 * k), 8 * k)
            pygame.draw.circle(s, col("#a86e3a"), (cx * k, 38 * k), 6 * k)
        for cx in (39, 61):
            pygame.draw.circle(s, WHITE, (cx * k, 50 * k), 7 * k)
        for cx in (41, 63):
            pygame.draw.circle(s, hexc("#141008"), (cx * k, 52 * k), 3.4 * k)

    def _render_hammer(self):
        k = 4
        s = pygame.Surface((52 * k, 52 * k), pygame.SRCALPHA)
        R = lambda x, y, w, h: pygame.Rect(x * k, y * k, w * k, h * k)
        pygame.draw.rect(s, hexc("#120c06"), R(20, 14, 12, 30), border_radius=3 * k)
        pygame.draw.rect(s, hexc("#b07a40"), R(22, 16, 8, 26), border_radius=2 * k)
        pygame.draw.rect(s, hexc("#ce9858"), R(23, 18, 3, 16))
        pygame.draw.rect(s, hexc("#120c06"), R(6, 2, 40, 18), border_radius=5 * k)
        pygame.draw.rect(s, hexc("#b6bac0"), R(9, 5, 34, 12), border_radius=4 * k)
        pygame.draw.rect(s, hexc("#d6dadf"), R(12, 7, 14, 5), border_radius=2 * k)
        return s

    def _box_glow(self, size, rgb, alpha, spread):
        w, h = size
        s = pygame.Surface((w + 2 * spread, h + 2 * spread), pygame.SRCALPHA)
        pygame.draw.rect(s, rgb + (alpha,), (spread // 2, spread // 2, w + spread, h + spread),
                         border_radius=self.L.P(10))
        s = blur(s, max(2, spread // 3))
        return s

    def _perimeter(self, rect, count):
        inset = self.L.P(1.5)
        x0, y0 = rect.left + inset, rect.top + inset
        w, h = rect.w - 2 * inset, rect.h - 2 * inset
        per = 2 * (w + h)
        pts = []
        for i in range(count):
            d = i / count * per
            if d < w:
                x, y = d, 0
            elif d < w + h:
                x, y = w, d - w
            elif d < 2 * w + h:
                x, y = w - (d - w - h), h
            else:
                x, y = 0, h - (d - 2 * w - h)
            pts.append((x0 + x, y0 + y, i * 1.8 / count))
        return pts

    # ----------------------------------------------------------------- input
    def cell_at(self, pos):
        for i, r in enumerate(self.L.cells):
            if r.collidepoint(pos):
                return i
        return None

    def update_input(self, now):
        """Sensor handling copied from the team's working game file:
        sensor input takes priority when a complete frame is available,
        otherwise the mouse is used."""
        try:
            mouse_focused = pygame.mouse.get_focused()
            mouse_pos = pygame.mouse.get_pos()
        except Exception:
            mouse_focused = False
            mouse_pos = (0, 0)

        if self.reader is not None:
            distances = self.reader.read_distances()
            if distances is not None:
                if "x" in distances and "y" in distances:
                    self.sensor_cell = sensor_cell_from_position(distances["x"], distances["y"])
                    self.sensor_position = (distances["x"], distances["y"])
                    self.sensor_last_update = pygame.time.get_ticks()
                    print(f"Sensor position: cell={self.sensor_cell} (x={distances['x']:.3f}, y={distances['y']:.3f})")
                else:
                    self.sensor_position = None
                    left, right = combine_left_right(distances)
                    fraction_x = sensor_fraction_x(left, right)
                    if fraction_x is not None:
                        row = sensor_fraction_row(left, right)
                        self.sensor_cell = sensor_cell_from_fraction(fraction_x, row)
                        self.sensor_last_update = pygame.time.get_ticks()
                        print(f"Sensor position: cell={self.sensor_cell} (left={left}, right={right})")

        grid = self.L.grid
        mouse_grid_cell = self.cell_at(mouse_pos)
        cell = self.sensor_cell if self.sensor_cell is not None else mouse_grid_cell

        control_pos = mouse_pos
        if self.sensor_position is not None:
            control_pos = (
                int(grid.x + self.sensor_position[0] * grid.w),
                int(grid.y + self.sensor_position[1] * grid.h),
            )
        elif self.sensor_cell is not None:
            control_pos = self.L.cells[self.sensor_cell].center

        if now - self.sensor_last_update > SENSOR_TIMEOUT_MS:
            self.sensor_cell = None
            self.sensor_position = None

        # The hammer is shown whenever there's live sensor input or the
        # window has the mouse, same rule as the team's file.
        show_hammer = self.sensor_cell is not None or mouse_focused
        pygame.mouse.set_visible(not show_hammer)
        self.cursor_cell = cell
        self.cursor_pos = control_pos if show_hammer else None

    def menu_action(self):
        cell = self.cursor_cell
        if self.state == "menu" and cell is not None and self.L.title.collidepoint(self.L.cells[cell].center):
            return None
        return MENU_CELL_ACTIONS.get(cell)

    # ----------------------------------------------------------------- logic
    def update(self, now):
        self.update_input(now)
        if self.state == "menu":
            self.update_menu(now)
        elif self.state == "play":
            self.update_play(now)
        elif self.state == "results":
            if now - self.results_start >= RESULTS_MS:
                self.back_to_menu(now)

    def update_menu(self, now):
        if self.blocked_cell is not None and self.cursor_cell != self.blocked_cell:
            self.blocked_cell = None
        action = self.menu_action()
        if action in DIFFICULTY_PRESETS and self.cursor_cell != self.blocked_cell:
            if self.charging != action:
                self.charging, self.charge_start = action, now
            elif now - self.charge_start >= MENU_CONFIRM_MS:
                self.hammer_anim = ("swing_charged", now)
                self.start_game(action, now)
        else:
            self.charging = None

    def start_game(self, difficulty, now):
        self.difficulty = difficulty
        self.wind_cell = None
        self.charging = None
        self.score = self.moles_popped = self.moles_hit = 0
        for m in self.moles:
            m.reset(now)
        self.bursts.clear()
        self.spawning = True
        self.next_spawn = now + DIFFICULTY_PRESETS[difficulty][0]
        self.end_at = None
        self.state = "play"

    def update_play(self, now):
        spawn_ms, up_ms = DIFFICULTY_PRESETS[self.difficulty]

        while self.spawning and now >= self.next_spawn:
            self.next_spawn += spawn_ms
            self.pop_mole(now, up_ms)

        for i, m in enumerate(self.moles):
            if m.up and m.hit and now - m.hit_at >= 260:
                m.reset(now)
            elif m.up and not m.hit and now >= m.down_at:
                m.reset(now)

        # Hit timing from the team's file: the hammer winds up when you
        # enter a cell with a live mole, and the hit lands after
        # HAMMER_WIND_MS if you're still there. Leaving cancels it.
        cell = self.cursor_cell
        live = cell is not None and self.moles[cell].up and not self.moles[cell].hit
        if not live:
            self.wind_cell = None
        elif self.wind_cell != cell:
            self.wind_cell, self.wind_start = cell, now
        elif now - self.wind_start >= HAMMER_WIND_MS:
            self.wind_cell = None
            m = self.moles[cell]
            if m.up and not m.hit:
                m.hit, m.hit_at = True, now
                m.move_to(Mole.HIT, now)
                self.score += POINTS_PER_HIT
                self.moles_hit += 1
                self.score_bump_at = now
                self.bursts[cell] = now
                self.hammer_anim = ("swing", now)

        if self.end_at is not None and now >= self.end_at:
            self.end_game(now)

    def pop_mole(self, now, up_ms):
        idle = [m for m in self.moles if not m.up]
        if not idle:
            return
        m = random.choice(idle)
        m.up, m.hit, m.down_at = True, False, now + up_ms
        m.move_to(Mole.UP, now)
        self.moles_popped += 1
        if self.moles_popped >= ROUND_MOLES:
            self.spawning = False
            self.end_at = now + up_ms + 400   # let the last mole finish first

    def end_game(self, now):
        self.spawning = False
        self.end_at = None
        for m in self.moles:
            m.up = False
            m.move_to(Mole.DOWN, now)
        self.state = "results"
        self.results_start = now

    def back_to_menu(self, now):
        self.state = "menu"
        self.wind_cell = None
        self.spawning = False
        self.end_at = None
        self.charging = None
        for m in self.moles:
            m.reset(now)
        # whoever is standing on a box must step off before it can trigger again
        self.blocked_cell = self.cursor_cell

    # ------------------------------------------------------------- drawing
    def draw(self, now):
        L = self.L
        self.screen.blit(self.static_bg, (0, 0))
        self.screen.set_clip(L.field)

        self.draw_hud(now)
        if self.state == "menu":
            self.draw_menu(now)
        else:
            self.draw_play(now)
            self.draw_menu_button()
        hammer_in_field = self.cursor_pos is not None and L.field.collidepoint(self.cursor_pos)
        if hammer_in_field:
            self.draw_hammer(now)
        if self.state == "results":
            self.draw_results(now)

        self.screen.blit(self.crt_overlay, L.field.topleft)
        self.screen.set_clip(None)
        if not hammer_in_field:
            self.draw_hammer(now)     # mouse outside the screen area: hammer still follows it

    def draw_hud(self, now):
        L = self.L
        y_mid = L.hud.centery
        label_score = self.text("pixel", 9, "SCORE", BLUE_LT)
        label_moles = self.text("pixel", 9, "MOLES", BLUE_LT)
        glow = (255, 255, 0, 115)

        bump = min(1.0, (now - self.score_bump_at) / 250.0)
        colour = lerp_colour(WHITE, AMBER, EASE_OUT(bump))
        score_surf = self.text("led", 30, f"{self.score:04d}", colour, glow)
        if bump < 1:
            score_surf = pygame.transform.rotozoom(score_surf, 0, lerp(1.25, 1.0, EASE_OUT(bump)))
        moles_surf = self.text("led", 30, f"{self.moles_popped:02d}/{ROUND_MOLES}", AMBER, glow)

        self.screen.blit(label_score, label_score.get_rect(midleft=(L.hud.left, y_mid + L.P(4))))
        self.blit_center(score_surf, (L.hud.left + label_score.get_width() + L.P(8) + L.P(30), y_mid))
        mx = L.hud.right - L.P(52)
        self.blit_center(moles_surf, (mx, y_mid))
        self.screen.blit(label_moles, label_moles.get_rect(midright=(mx - L.P(36), y_mid + L.P(4))))

    def draw_chase_dots(self, dots, now, running, base_alpha=.12):
        t = now / 1000.0
        for x, y, delay in dots:
            if running:
                phase = ((t - delay) % 1.8) / 1.8
                b = .12 + .88 * (1 - abs(2 * phase - 1))
            else:
                b = base_alpha
            if b > .5:
                self.dot_halo.set_alpha(int(255 * (b - .5) * 2 * .9))
                self.blit_center(self.dot_halo, (x, y))
            self.dot_core.set_alpha(int(255 * b))
            self.blit_center(self.dot_core, (x, y))

    def draw_menu(self, now):
        L, P = self.L, self.L.P
        action = self.menu_action()

        # title banner
        pygame.draw.rect(self.screen, PANEL, L.title, border_radius=P(6))
        pygame.draw.rect(self.screen, MAZE_BLUE, L.title, P(3), border_radius=P(6))
        if self.logo:
            self.blit_center(self.logo, L.title.center)
        else:
            self.blit_center(self.text("pixel", 22, "WACK A MOLE", CREAM), L.title.center)
        self.draw_chase_dots(self.title_dots, now, True)

        # difficulty boxes
        for idx, name in ((3, "Easy"), (4, "Medium"), (5, "Hard")):
            r = L.cells[idx]
            charging = self.charging == name
            if charging:
                self.blit_center(self.glow_charge, r.center)
            pygame.draw.rect(self.screen, PANEL, r, border_radius=P(6))
            pygame.draw.rect(self.screen, DIFF_COLOURS[name], r, P(3), border_radius=P(6))
            self.blit_center(self.text("pixel", 13, name.upper(), DIFF_COLOURS[name]), r.center)
            self.draw_chase_dots(self.diff_dots[idx], now, charging, base_alpha=.1)

        # bottom row: blank / help / blank
        for idx in (6, 8):
            r = L.cells[idx]
            pygame.draw.rect(self.screen, PANEL, r, border_radius=P(6))
            pygame.draw.rect(self.screen, MAZE_BLUE, r, P(3), border_radius=P(6))
            dot = pygame.Surface((P(6) + 2, P(6) + 2), pygame.SRCALPHA)
            pygame.draw.circle(dot, DOT[:3] + (128,), (dot.get_width() // 2,) * 2, max(2, P(3)))
            self.blit_center(dot, r.center)

        r = L.cells[7]
        hovering = action == "help"
        colour = AMBER if hovering else CREAM
        if hovering:
            self.blit_center(self.glow_help, r.center)
        pygame.draw.rect(self.screen, PANEL, r, border_radius=P(6))
        pygame.draw.rect(self.screen, AMBER if hovering else MAZE_BLUE, r, P(3), border_radius=P(6))
        self.blit_center(self.text("pixel", 22, "?", colour), (r.centerx, r.centery - P(8)))
        self.blit_center(self.text("pixel", 9, "HELP", colour), (r.centerx, r.centery + P(16)))
        if hovering:
            self.draw_help_tooltip(r)

    def draw_help_tooltip(self, cell):
        P = self.L.P
        width = min(P(320), self.L.grid.w - P(20))
        body = self.font("led", 16)
        lines = []
        for item in ("Move left/right and closer/further from the sensors (or move the mouse) to steer the hammer.",
                     "Stand on a mole's square as it pops up to whack it.",
                     "Hold on Easy, Medium or Hard for 2 seconds to start.",
                     f"Each round is {ROUND_MOLES} moles. Every hit is worth {POINTS_PER_HIT} points."):
            wrapped = wrap_text(body, item, width - P(44))
            lines.append(("bullet", wrapped[0]))
            lines += [("cont", w) for w in wrapped[1:]]
        heading = self.text("pixel", 9, "HOW TO PLAY", AMBER)
        line_h = body.get_linesize()
        height = P(12) + heading.get_height() + P(8) + line_h * len(lines) + P(12)
        box = pygame.Rect(0, 0, width, height)
        box.midbottom = (cell.centerx, cell.top - P(8))
        box.clamp_ip(self.L.field.inflate(-P(6), -P(6)))

        pygame.draw.rect(self.screen, PANEL, box, border_radius=P(6))
        pygame.draw.rect(self.screen, AMBER, box, P(2), border_radius=P(6))
        tip = [(cell.centerx - P(6), box.bottom), (cell.centerx + P(6), box.bottom), (cell.centerx, box.bottom + P(6))]
        pygame.draw.polygon(self.screen, AMBER, tip)

        y = box.top + P(12)
        self.screen.blit(heading, (box.left + P(14), y))
        y += heading.get_height() + P(8)
        for kind, line in lines:
            if kind == "bullet":
                pygame.draw.circle(self.screen, CREAM, (box.left + P(20), y + line_h // 2), max(2, P(2)))
            self.screen.blit(body.render(line, True, CREAM[:3]), (box.left + P(30), y))
            y += line_h

    def draw_play(self, now):
        L = self.L
        k = L.cell_size / 100.0
        for i, r in enumerate(L.cells):
            self.screen.blit(self.hole_art, r.topleft)
            m = self.moles[i]
            off = m.offset(now)
            if off < Mole.DOWN - .5:
                clip = pygame.Rect(r.left, r.top - int(40 * k), r.w, int(87 * k)).clip(L.field)
                self.screen.set_clip(clip)
                art = self.mole_hit_art if m.hit else self.mole_art
                self.screen.blit(art, (r.left, r.top + int(off * k)))
                self.screen.set_clip(L.field)

            started = self.bursts.get(i)
            if started is not None:
                p = (now - started) / 550.0
                if p >= 1:
                    del self.bursts[i]
                else:
                    surf = self.text("pixel", 11, "+10", AMBER, (255, 255, 0, 230)).copy()
                    surf.set_alpha(int(255 * (1 - p)))
                    self.blit_center(surf, (r.centerx, r.centery - L.P(26) * EASE_OUT(p)))

    def menu_button_rect(self):
        P = self.L.P
        label = self.text("pixel", 7, "MENU", CREAM)
        r = label.get_rect().inflate(P(12), P(8))
        r.topright = (self.L.field.right - P(6), self.L.field.top + P(6))
        return r, label

    def draw_menu_button(self):
        r, _ = self.menu_button_rect()
        hover = r.collidepoint(pygame.mouse.get_pos())
        colour = AMBER if hover else CREAM
        pygame.draw.rect(self.screen, PANEL, r, border_radius=self.L.P(4))
        pygame.draw.rect(self.screen, AMBER if hover else MAZE_BLUE, r, 1, border_radius=self.L.P(4))
        self.blit_center(self.text("pixel", 7, "MENU", colour), r.center)

    def hammer_angle(self, now):
        def keyframes(frames, elapsed, total):
            p = elapsed / total
            for (t0, a0), (t1, a1) in zip(frames, frames[1:]):
                if p <= t1:
                    return lerp(a0, a1, EASE_OUT((p - t0) / (t1 - t0)))
            return frames[-1][1]

        if self.hammer_anim:
            kind, start = self.hammer_anim
            elapsed = now - start
            if kind == "swing" and elapsed < 220:
                return keyframes([(0, 32), (.45, -34), (1, 32)], elapsed, 220)
            if kind == "swing_charged" and elapsed < 260:
                return keyframes([(0, 95), (.4, -42), (1, 32)], elapsed, 260)
            self.hammer_anim = None
        if self.charging:
            return lerp(32, 95, min(1.0, (now - self.charge_start) / MENU_CONFIRM_MS))
        if self.wind_cell is not None:
            return lerp(32, 95, min(1.0, (now - self.wind_start) / HAMMER_WIND_MS))
        return 32

    def draw_hammer(self, now):
        if self.cursor_pos is None:
            return
        size = self.L.P(42)
        zoom = size / self.hammer_art.get_width()
        angle = self.hammer_angle(now)
        img = pygame.transform.rotozoom(self.hammer_art, angle, zoom)
        shadow = img.copy()
        shadow.fill((0, 0, 0, 128), special_flags=pygame.BLEND_RGBA_MIN)
        x, y = self.cursor_pos
        self.blit_center(shadow, (x, y + self.L.P(2)))
        self.blit_center(img, (x, y))

    def draw_results(self, now):
        L, P = self.L, self.L.P
        elapsed = now - self.results_start
        fade = min(1.0, elapsed / 200.0)

        shade = pygame.Surface(L.field.size, pygame.SRCALPHA)
        shade.fill((0, 0, 0, int(184 * fade)))
        self.screen.blit(shade, L.field.topleft)

        width = min(P(300), int(L.field.w * .8))
        title = self.text("pixel", 16, "GAME OVER", RED_LED, (255, 0, 0, 150))
        big = self.text("led", 56, str(self.moles_hit), AMBER, (255, 255, 0, 128))
        line = self.text("led", 20, f"of {ROUND_MOLES} moles whacked", CREAM)
        score = self.text("led", 20, f"Score {self.score}", BLUE_LT)
        secs = max(0, math.ceil((RESULTS_MS - elapsed) / 1000))
        hint = self.text("pixel", 8, f"MENU IN {secs}", BLUE_LT)
        bar_h = P(6)

        parts = [(title, P(4)), (big, -P(8)), (line, P(2)), (score, P(14)), (None, P(8)), (hint, 0)]
        height = P(18) + sum((s.get_height() if s else bar_h) + gap for s, gap in parts) + P(14)

        panel = pygame.Surface((width + P(40), height + P(40)), pygame.SRCALPHA)
        pr = pygame.Rect(P(20), P(20), width, height)
        glow = pygame.Surface(panel.get_size(), pygame.SRCALPHA)
        pygame.draw.rect(glow, (255, 255, 0, 80), pr.inflate(P(10), P(10)), border_radius=P(10))
        panel.blit(blur(glow, 6), (0, 0))
        pygame.draw.rect(panel, PANEL, pr, border_radius=P(6))
        pygame.draw.rect(panel, AMBER, pr, P(3), border_radius=P(6))

        y = pr.top + P(18)
        for surf, gap in parts:
            if surf is None:
                bar = pygame.Rect(pr.left + P(16), y, pr.w - P(32), bar_h)
                left = max(0.0, 1 - elapsed / RESULTS_MS)
                fill = bar.copy()
                fill.w = int(bar.w * left)
                pygame.draw.rect(panel, MAZE_BLUE_LT, fill, border_radius=P(3))
                pygame.draw.rect(panel, MAZE_BLUE, bar, 1, border_radius=P(3))
                y += bar_h + gap
            else:
                panel.blit(surf, surf.get_rect(midtop=(pr.centerx, y)))
                y += surf.get_height() + gap

        pop = lerp(.85, 1.0, EASE_BACK(min(1.0, elapsed / 250.0)))
        if pop != 1.0:
            panel = pygame.transform.rotozoom(panel, 0, pop)
        panel.set_alpha(int(255 * fade))
        self.blit_center(panel, L.field.center)

    # --------------------------------------------------------------- main loop
    def run(self):
        running = True
        while running:
            now = pygame.time.get_ticks()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        if self.state == "menu":
                            running = False
                        else:
                            self.back_to_menu(now)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if self.state != "menu" and self.menu_button_rect()[0].collidepoint(event.pos):
                        self.back_to_menu(now)

            self.update(now)
            self.draw(now)
            pygame.display.flip()
            self.clock.tick(FPS)

        if self.reader is not None:
            self.reader.close()
        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="Whack-a-Mole arcade cabinet")
    parser.add_argument("--scale", type=float, default=UI_SCALE, help="window size multiplier (default %(default)s)")
    args = parser.parse_args()
    Game(args.scale).run()


if __name__ == "__main__":
    sys.exit(main())
