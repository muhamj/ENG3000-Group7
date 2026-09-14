"""
Whack-a-Mole — Redesigned Arcade UI (Pygame)
=============================================

This ports the browser mockup's visual design into Pygame and wires the
player cursor to REAL sensor data instead of the mouse.

HOW POSITION IS COMPUTED
-------------------------
Your two ultrasonic sensors sit on a known baseline (per your Figure 3
geometry: ~0.96 m apart, mounted at ~39 degrees). Each sensor reports a
straight-line distance to the player. Two distances + a known baseline is
enough to triangulate an (x, y) position (this is standard trilateration —
the same idea as GPS, just in 2D with two anchors instead of three+).

    Sensor A --- baseline (0.96 m) --- Sensor B
         \\                              /
        dA \\                          / dB
             \\                      /
                \\                /
                   *  player  *

WHAT YOU NEED TO CHANGE
-------------------------
1. SERIAL_PORT / BAUD_RATE below — match your ESP32's COM port.
2. parse_serial_line() — I've assumed lines look like:
       "S1: Distance: 9.40 cm"
       "S2: Distance: 32.74 cm"
   Your raw serial log (Figure 4) only showed unlabeled "Distance: X cm"
   lines, so I don't know for certain how your reader tells the two
   sensors apart yet. If your format differs, this is the ONLY function
   you should need to edit — everything else (triangulation, rendering,
   game logic) stays the same.
3. SENSOR_BASELINE_M / DEAD_ZONE_M — pull these from your final Electrical
   specs if they change.

If no serial device is found, the game automatically falls back to mouse
control so you can keep testing the UI without hardware plugged in.
Press M at any time to force mouse mode manually.
"""

import math
import os
import random
import re
import sys
import threading
import time

import pygame

LOGO_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "logo.png")
FONT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets", "fonts")
PIXEL_FONT_PATH = os.path.join(FONT_DIR, "PressStart2P.ttf")   # chunky arcade marquee font
LED_FONT_PATH = os.path.join(FONT_DIR, "VT323.ttf")            # CRT/LED terminal font

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# ----------------------------------------------------------------------------
# CONFIG — adjust these to match your team's confirmed specs
# ----------------------------------------------------------------------------
SERIAL_PORT = "COM5"          # <-- change to your ESP32's port ("/dev/ttyUSB0" on Linux/Mac)
BAUD_RATE = 115200

PLAY_AREA_W_M = 1.5            # playing-area width (Section 3 param)
PLAY_AREA_D_M = 1.4            # playing-area depth (Section 3 param)
SENSOR_BASELINE_M = 0.96       # distance between the two sensors (Figure 3)
DEAD_ZONE_M = 0.60             # dead-zone alarm trigger distance (Section 3 param)
MAX_RELIABLE_RANGE_M = 1.65    # sensor reliability limit found in testing (Figure 4)

SCREEN_W, SCREEN_H = 600, 660
FPS = 60

# Colors — classic Pac-Man arcade palette: black maze background, neon-blue
# maze walls, and the four ghost colors used as accents throughout.
COL_BG = (0, 0, 0)
COL_CAB = (8, 8, 14)
COL_CAB_DK = (0, 0, 0)
COL_MAZE_BLUE = (33, 33, 222)
COL_MAZE_BLUE_DK = (14, 14, 92)
COL_PANEL = (0, 0, 0)
COL_SCREEN_BG = (0, 0, 0)
COL_MOUND = (92, 63, 39)
COL_MOUND_DARK = (61, 41, 23)
COL_HOLE = (4, 3, 2)
COL_CREAM = (255, 224, 90)          # warm yellow, used for most HUD/help text now
COL_PAC_YELLOW = (255, 255, 0)
COL_GHOST_RED = (255, 0, 0)
COL_BLUE_LT = (110, 150, 255)        # lighter blue accent (used for "Easy")
COL_BLUE_DEEP = (40, 60, 210)        # deep blue accent (used for "Hard") — brighter than the maze walls for contrast against black
COL_AMBER = COL_PAC_YELLOW          # kept as an alias so older references still read correctly
COL_AMBER_DEEP = (196, 196, 0)
COL_ALARM = COL_GHOST_RED
COL_GREEN_LED = (255, 255, 255)     # score digits — classic Pac-Man HUD is white pixel text
COL_MOLE_BODY = (168, 110, 58)
COL_MOLE_BODY_LT = (206, 148, 88)
COL_DOT = (255, 184, 151)           # pac-dot pink-white
COL_BULB_OFF = (30, 30, 30)

GRID_ROWS, GRID_COLS = 3, 3

# ----------------------------------------------------------------------------
# DIFFICULTY LEVELS
# ----------------------------------------------------------------------------
# "Hard" reproduces the original single-speed behaviour you had before
# (fast spawns, short up-time). Easy/Medium are slower variants of the same
# per-level scaling so the game still speeds up as you score, just from a
# gentler starting point.
DIFFICULTY_PRESETS = {
    "Easy":   {"base_spawn": 1.80, "spawn_decay": 0.05, "min_spawn": 1.00,
               "base_dur": 2.40,  "dur_decay": 0.12,   "min_dur": 1.40},
    "Medium": {"base_spawn": 1.30, "spawn_decay": 0.07, "min_spawn": 0.70,
               "base_dur": 1.90,  "dur_decay": 0.15,   "min_dur": 1.00},
    "Hard":   {"base_spawn": 0.90, "spawn_decay": 0.08, "min_spawn": 0.35,
               "base_dur": 1.50,  "dur_decay": 0.18,   "min_dur": 0.60},
}
DIFFICULTY_ORDER = ["Easy", "Medium", "Hard"]


# ----------------------------------------------------------------------------
# SENSOR INPUT
# ----------------------------------------------------------------------------
class SensorReader:
    """Reads two ultrasonic distances over serial and triangulates a position.

    Falls back to mouse control if no serial device is available/opened.
    """

    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        self.dist_a = None   # meters
        self.dist_b = None   # meters
        self.lock = threading.Lock()
        self.use_mouse = True
        self.ser = None

        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.2)
                self.use_mouse = False
                self.thread = threading.Thread(target=self._read_loop, daemon=True)
                self.thread.start()
                print(f"[SensorReader] Connected to {port} — using live sensor data.")
            except Exception as e:
                print(f"[SensorReader] Could not open {port} ({e}). Falling back to mouse.")
        else:
            print("[SensorReader] pyserial not installed. Falling back to mouse. "
                  "Install with: pip install pyserial")

    def _read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                sensor_id, dist_cm = parse_serial_line(line)
                if sensor_id is None:
                    continue
                with self.lock:
                    if sensor_id == "A":
                        self.dist_a = dist_cm / 100.0
                    elif sensor_id == "B":
                        self.dist_b = dist_cm / 100.0
            except Exception:
                time.sleep(0.05)

    def get_position_m(self, mouse_xy_norm):
        """Returns (x_m, y_m) within the playing area, or None if unreliable.

        mouse_xy_norm: (nx, ny) in [0,1] from the mouse, used as a fallback.
        """
        if self.use_mouse:
            nx, ny = mouse_xy_norm
            return nx * PLAY_AREA_W_M, ny * PLAY_AREA_D_M

        with self.lock:
            dA, dB = self.dist_a, self.dist_b

        if dA is None or dB is None:
            return None
        if dA > MAX_RELIABLE_RANGE_M or dB > MAX_RELIABLE_RANGE_M:
            return None  # beyond the reliable range found in your testing

        return triangulate(dA, dB, SENSOR_BASELINE_M)


def parse_serial_line(line):
    """EDIT THIS to match your actual serial format.

    Assumed default (adjust once your team's format is confirmed):
        "S1: Distance: 9.40 cm"  -> returns ("A", 9.40)
        "S2: Distance: 32.74 cm" -> returns ("B", 32.74)
    """
    m = re.search(r"S1.*?Distance:\s*([\d.]+)\s*cm", line, re.IGNORECASE)
    if m:
        return "A", float(m.group(1))
    m = re.search(r"S2.*?Distance:\s*([\d.]+)\s*cm", line, re.IGNORECASE)
    if m:
        return "B", float(m.group(1))
    return None, None


def triangulate(dA, dB, baseline):
    """2D trilateration: sensor A at (0,0), sensor B at (baseline,0).

    Solves for the point (x, y) that is dA from A and dB from B.
    Returns (x, y) in meters, with y measured out into the playing area.
    """
    x = (dA ** 2 - dB ** 2 + baseline ** 2) / (2 * baseline)
    y_sq = dA ** 2 - x ** 2
    if y_sq < 0:
        return None
    y = math.sqrt(y_sq)
    return x, y


# ----------------------------------------------------------------------------
# GAME
# ----------------------------------------------------------------------------
class Mole:
    def __init__(self, rect, row=0, col=0):
        self.rect = rect
        self.row = row
        self.col = col
        self.up = False
        self.hit = False
        self.pop_progress = 0.0  # 0 = hidden, 1 = fully up
        self.up_timer = 0.0
        self.up_duration = 1.2

    def pop_up(self, duration):
        self.up = True
        self.hit = False
        self.up_timer = 0.0
        self.up_duration = duration

    def update(self, dt):
        target = 1.0 if (self.up and not self.hit) else 0.0
        self.pop_progress += (target - self.pop_progress) * min(1, dt * 12)

        if self.hit:
            if self.pop_progress <= 0.02:
                self.up = False
                self.hit = False
                self.pop_progress = 0.0
            return

        if self.up:
            self.up_timer += dt
            if self.up_timer > self.up_duration:
                self.up = False

    def contains(self, px, py):
        return self.up and not self.hit and self.rect.collidepoint(px, py)


def draw_mole(surface, hole_rect, progress):
    """Draws a ghost-silhouette mole rising up out of `hole_rect` — same
    flat-color, thick-outline, "arcade sprite" language as the Pac-Man
    cursor and maze walls, just recoloured mole-brown with small ears/
    whiskers kept so it still reads as a mole, not a literal ghost."""
    if progress < 0.02:
        return

    w = hole_rect.width * 1.3
    h = hole_rect.height * 3.1
    cx = hole_rect.centerx
    bottom_y = hole_rect.centery + hole_rect.height * 0.35
    top_y = bottom_y - h * progress * 1.05

    clip_rect = pygame.Rect(hole_rect.left - w, hole_rect.top - h, w * 3, h + hole_rect.height * 0.55)
    old_clip = surface.get_clip()
    surface.set_clip(clip_rect)

    radius = w / 2
    dome_cy = top_y + radius
    skirt_h = h * 0.20
    body_bottom = top_y + h * 0.82

    # Dome (top half-circle), traced left -> over the top -> right
    points = []
    steps = 10
    for i in range(steps + 1):
        theta = math.radians(180 - (180 * i / steps))
        points.append((cx + radius * math.cos(theta), dome_cy - radius * math.sin(theta)))

    # Zigzag "ghost skirt" bottom edge, right -> left
    n_humps = 3
    seg_w = (2 * radius) / n_humps
    x_right = cx + radius
    for i in range(n_humps):
        x_start = x_right - i * seg_w
        x_mid = x_start - seg_w / 2
        points.append((x_start, body_bottom))
        points.append((x_mid, body_bottom + skirt_h))
    points.append((cx - radius, body_bottom))

    outline_col = (18, 12, 6)
    pygame.draw.polygon(surface, outline_col, points)
    inset_pts = [(cx + (px - cx) * 0.92, dome_cy + (py - dome_cy) * 0.94) for px, py in points]
    pygame.draw.polygon(surface, COL_MOLE_BODY, inset_pts)

    # Flat highlight patch (no gradients — matches the maze's flat-color style)
    hi_rect = pygame.Rect(0, 0, w * 0.5, h * 0.22)
    hi_rect.center = (cx - w * 0.12, dome_cy - radius * 0.25)
    pygame.draw.ellipse(surface, COL_MOLE_BODY_LT, hi_rect)

    # Big flat-shaded eyes, Pac-Man-ghost style (white with black pupils)
    eye_y = dome_cy - radius * 0.15
    eye_r = max(3, int(w * 0.11))
    for dx in (-w * 0.16, w * 0.16):
        pygame.draw.circle(surface, (255, 255, 255), (int(cx + dx), int(eye_y)), eye_r)
        pygame.draw.circle(surface, (20, 15, 10), (int(cx + dx + eye_r * 0.25), int(eye_y + eye_r * 0.35)), max(2, eye_r // 2))

    # Small ears + whiskers so it still reads as a mole, not a literal ghost
    for dx in (-w * 0.30, w * 0.30):
        pygame.draw.circle(surface, COL_MOLE_BODY, (int(cx + dx), int(dome_cy - radius * 0.78)), max(3, int(w * 0.08)))
        pygame.draw.circle(surface, outline_col, (int(cx + dx), int(dome_cy - radius * 0.78)), max(3, int(w * 0.08)), 1)
    whisk_y = dome_cy + radius * 0.32
    for side in (-1, 1):
        for wy in (-4, 0, 4):
            pygame.draw.line(surface, (245, 245, 245),
                              (cx + side * w * 0.10, whisk_y + wy),
                              (cx + side * w * 0.34, whisk_y + wy * 1.4), 1)

    surface.set_clip(old_clip)


def make_scanline_overlay(w, h):
    """Pre-rendered CRT scanline + vignette overlay, blitted once per frame."""
    surf = pygame.Surface((w, h), pygame.SRCALPHA)
    for y in range(0, h, 3):
        pygame.draw.line(surf, (0, 0, 0, 45), (0, y), (w, y))
    # simple vignette via nested faded rects
    steps = 18
    for i in range(steps):
        alpha = int(70 * (i / steps) ** 2)
        pad = i * (min(w, h) // (steps * 2))
        pygame.draw.rect(surf, (0, 0, 0, alpha), (pad, pad, w - pad * 2, h - pad * 2),
                          width=max(1, min(w, h) // (steps * 2) + 1), border_radius=6)
    return surf


def draw_pacman(surface, cx, cy, radius, mouth_frac, color=(255, 255, 0)):
    """Draws a classic Pac-Man shape (circle with a wedge mouth cut out)
    onto `surface` at (cx, cy). mouth_frac in [0,1] controls how open it is."""
    d = radius * 2 + 4
    temp = pygame.Surface((d, d), pygame.SRCALPHA)
    pygame.draw.circle(temp, color, (d // 2, d // 2), radius)
    angle = 8 + mouth_frac * 30  # degrees of mouth opening, each side
    if angle > 0.5:
        p1 = (d // 2, d // 2)
        pts = [p1]
        for a in (-angle, angle):
            rad = math.radians(a)
            pts.append((d // 2 + radius * 1.5 * math.cos(rad), d // 2 + radius * 1.5 * math.sin(rad)))
        pygame.draw.polygon(temp, (0, 0, 0, 0), pts)
    # tiny eye
    pygame.draw.circle(temp, (30, 20, 10), (int(d * 0.52), int(d * 0.28)), max(1, radius // 7))
    surface.blit(temp, (cx - d // 2, cy - d // 2))


def draw_hammer(surface, cx, cy, swing_progress, danger=False, windup_progress=0.0):
    """Draws a flat-shaded pixel-art hammer, matching the maze's outlined-
    flat-color style. swing_progress in [0,1]: 0/1 = resting, 0.5 = mid-swing
    (driven by a sine curve so it snaps down and springs back on a hit).
    windup_progress in [0,1]: while dwelling on a menu option, the hammer
    gradually rotates further back as it "charges up" toward a selection."""
    size = 52
    temp = pygame.Surface((size, size), pygame.SRCALPHA)
    outline = (255, 60, 48) if danger else (18, 12, 6)
    handle_color = (176, 122, 64)
    handle_hi = (206, 152, 88)
    head_color = (150, 60, 40) if danger else (182, 186, 192)
    head_hi = (196, 90, 70) if danger else (214, 218, 222)

    handle_rect = pygame.Rect(0, 0, 8, 32)
    handle_rect.midbottom = (size // 2, size - 2)
    pygame.draw.rect(temp, outline, handle_rect.inflate(5, 5), border_radius=3)
    pygame.draw.rect(temp, handle_color, handle_rect, border_radius=2)
    pygame.draw.rect(temp, handle_hi, handle_rect.inflate(-4, -18), border_radius=1)

    head_rect = pygame.Rect(0, 0, 30, 16)
    head_rect.midbottom = (size // 2, handle_rect.top + 8)
    pygame.draw.rect(temp, outline, head_rect.inflate(5, 5), border_radius=4)
    pygame.draw.rect(temp, head_color, head_rect, border_radius=3)
    pygame.draw.rect(temp, head_hi, head_rect.inflate(-6, -8), border_radius=2)

    base_tilt = -32 - windup_progress * 63   # winds back further as dwell progresses
    swing_amount = math.sin(swing_progress * math.pi)  # 0 -> 1 -> 0, snappy down-and-back
    angle = base_tilt + swing_amount * (66 + windup_progress * 40)
    rotated = pygame.transform.rotate(temp, angle)
    rect = rotated.get_rect(center=(cx, cy - 6))
    surface.blit(rotated, rect)


def draw_light_ring(surface, rect, frame_n, color=(255, 255, 0), count=14, off_color=(40, 40, 10)):
    """Draws a ring of small chase lights around `rect`'s border — the same
    visual language as the title marquee's bulb row, reused as hover
    feedback on the menu boxes."""
    perimeter_points = []
    steps_per_side = count // 4
    for i in range(steps_per_side):
        t = i / steps_per_side
        perimeter_points.append((rect.left + rect.width * t, rect.top))
    for i in range(steps_per_side):
        t = i / steps_per_side
        perimeter_points.append((rect.right, rect.top + rect.height * t))
    for i in range(steps_per_side):
        t = i / steps_per_side
        perimeter_points.append((rect.right - rect.width * t, rect.bottom))
    for i in range(steps_per_side):
        t = i / steps_per_side
        perimeter_points.append((rect.left, rect.bottom - rect.height * t))

    phase = (frame_n // 4) % 4
    for i, (px, py) in enumerate(perimeter_points):
        lit = (i % 4 == phase)
        pygame.draw.circle(surface, color if lit else off_color, (int(px), int(py)), 3)


def run():
    pygame.init()
    screen_w, screen_h = SCREEN_W, SCREEN_H
    screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
    pygame.display.set_caption("Whack-a-Mole — Arcade Cabinet")
    clock = pygame.time.Clock()

    def load_font(path, size, fallback_name="Arial", bold=False):
        if os.path.exists(path):
            return pygame.font.Font(path, size)
        return pygame.font.SysFont(fallback_name, size, bold=bold)

    font_pixel_lg = load_font(PIXEL_FONT_PATH, 15, bold=True)
    font_pixel_md = load_font(PIXEL_FONT_PATH, 10)
    font_pixel_sm = load_font(PIXEL_FONT_PATH, 8)
    font_pixel_xs = load_font(PIXEL_FONT_PATH, 16)   # big "?" glyph
    font_led_md = load_font(LED_FONT_PATH, 22)
    font_led_sm = load_font(LED_FONT_PATH, 18)

    sensor = SensorReader()

    logo_img_raw = None
    if os.path.exists(LOGO_PATH):
        logo_img_raw = pygame.image.load(LOGO_PATH).convert_alpha()
    else:
        print(f"[Assets] Logo not found at {LOGO_PATH} — falling back to text.")

    moles = [Mole(pygame.Rect(0, 0, 10, 10), r, c) for r in range(GRID_ROWS) for c in range(GRID_COLS)]

    def compute_layout(w, h):
        """Everything — title, difficulty picker, help, AND the mole grid —
        lives inside ONE 3x3 area now. No separate marquee/HUD/dead-zone
        bars; this single square is resized to fit whatever window space
        is available."""
        margin = 26
        top_pad = 34
        bottom_pad = 30
        available_h = h - top_pad - bottom_pad
        available_w = w - 2 * margin
        field_size = max(240, min(available_w, available_h))
        field_x = (w - field_size) // 2
        field_top = top_pad + max(0, (available_h - field_size) / 2)
        field_rect = pygame.Rect(field_x, int(field_top), field_size, field_size)

        logo_img = None
        if logo_img_raw:
            logo_w = int(field_size * 0.5)
            logo_h = int(logo_img_raw.get_height() * logo_w / logo_img_raw.get_width())
            logo_img = pygame.transform.smoothscale(logo_img_raw, (logo_w, logo_h))

        cell_w = field_rect.width / GRID_COLS
        cell_h = field_rect.height / GRID_ROWS
        for m in moles:
            m.rect = pygame.Rect(
                field_rect.left + m.col * cell_w + 8,
                field_rect.top + m.row * cell_h + 8,
                cell_w - 16, cell_h - 16,
            )

        bulb_count = 16
        bulb_span = field_rect.width * 0.86
        bulb_gap = bulb_span / (bulb_count - 1)
        bulb_start_x = field_rect.left + (field_rect.width - bulb_span) / 2

        return {
            "field_rect": field_rect, "logo_img": logo_img,
            "bulb_count": bulb_count, "bulb_gap": bulb_gap, "bulb_start_x": bulb_start_x,
            "scanline_overlay": make_scanline_overlay(field_rect.width, field_rect.height),
        }

    layout = compute_layout(screen_w, screen_h)

    def menu_boxes(field_rect):
        """Rects for the 6 interactive/decorative bottom-two-thirds cells of
        the menu screen: difficulty row + help row. Recomputed every frame
        (cheap) so resize is always consistent with what's drawn."""
        cell_w = field_rect.width / 3
        cell_h = field_rect.height / 3
        pad = 8
        boxes = {}
        for c, name in enumerate(DIFFICULTY_ORDER):
            boxes[name] = pygame.Rect(
                field_rect.left + c * cell_w + pad, field_rect.top + cell_h + pad,
                cell_w - 2 * pad, cell_h - 2 * pad,
            )
        boxes["help"] = pygame.Rect(
            field_rect.left + 1 * cell_w + pad, field_rect.top + 2 * cell_h + pad,
            cell_w - 2 * pad, cell_h - 2 * pad,
        )
        boxes["deco_left"] = pygame.Rect(
            field_rect.left + 0 * cell_w + pad, field_rect.top + 2 * cell_h + pad,
            cell_w - 2 * pad, cell_h - 2 * pad,
        )
        boxes["deco_right"] = pygame.Rect(
            field_rect.left + 2 * cell_w + pad, field_rect.top + 2 * cell_h + pad,
            cell_w - 2 * pad, cell_h - 2 * pad,
        )
        return boxes

    running = True
    game_started = False
    score = 0
    level = 1
    time_left = 60.0
    spawn_timer = 0.0
    force_mouse = sensor.use_mouse
    difficulty = "Medium"
    HAMMER_SWING_DURATION = 0.22
    hammer_swing_timer = 0.0
    DWELL_SECONDS = 2.0
    hover_target = None      # which menu/control box the hammer is currently over
    hover_elapsed = 0.0      # how long it's been hovering that same box
    menu_button_hover_time = 0.0

    screenshot_mode = bool(os.environ.get("SCREENSHOT_MODE"))
    overlay_screenshot = bool(os.environ.get("OVERLAY_SCREENSHOT"))
    diff_hover_screenshot = bool(os.environ.get("DIFF_HOVER_SCREENSHOT"))
    sensor_demo_mode = bool(os.environ.get("SENSOR_DEMO_MODE"))
    mole_preview_mode = bool(os.environ.get("MOLE_PREVIEW_MODE"))
    capture_gif_dir = os.environ.get("CAPTURE_GIF_DIR")
    capture_frames = int(os.environ.get("CAPTURE_FRAMES", "240"))
    frame_n = 0
    if screenshot_mode and not overlay_screenshot:
        game_started = True
        score, level, time_left = 40, 2, 47.0
        if mole_preview_mode:
            for i, m in enumerate(moles):
                if i % 2 == 0:
                    m.up = True
                    m.pop_progress = 1.0
        difficulty = "Hard"
    if capture_gif_dir:
        os.makedirs(capture_gif_dir, exist_ok=True)
        game_started = True
        sensor_demo_mode = True
        difficulty = "Hard"
        score, level, time_left = 0, 1, 60.0

    while running:
        dt = clock.tick(FPS) / 1000.0
        frame_n += 1
        field_rect = layout["field_rect"]
        boxes = menu_boxes(field_rect)
        menu_button_rect = pygame.Rect(field_rect.left + 18, field_rect.top + 18, 82, 28)
        exit_button_rect = pygame.Rect(field_rect.right - 100, field_rect.top + 18, 82, 28)

        if capture_gif_dir:
            mx, my = pygame.mouse.get_pos()
        elif screenshot_mode:
            if diff_hover_screenshot:
                mx, my = int(field_rect.left + field_rect.width * 0.5), int(field_rect.top + field_rect.height * 0.5)
            else:
                mx, my = int(field_rect.left + field_rect.width * 0.62), int(field_rect.top + field_rect.height * 0.68)
        else:
            mx, my = pygame.mouse.get_pos()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.VIDEORESIZE:
                screen_w, screen_h = event.w, event.h
                screen = pygame.display.set_mode((screen_w, screen_h), pygame.RESIZABLE)
                layout = compute_layout(screen_w, screen_h)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    force_mouse = not force_mouse
                elif not game_started and event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                    difficulty = DIFFICULTY_ORDER[event.key - pygame.K_1]
                    game_started = True
                    score, level, time_left, spawn_timer = 0, 1, 60.0, 0.0

        # ---- Determine player position (only meaningful once playing) ----
        if sensor_demo_mode:
            t = frame_n / FPS
            sim_x = PLAY_AREA_W_M * (0.5 + 0.28 * math.sin(t * 0.6))
            sim_y = PLAY_AREA_D_M * (0.55 + 0.22 * math.cos(t * 0.4))
            dA = math.hypot(sim_x, sim_y)
            dB = math.hypot(sim_x - SENSOR_BASELINE_M, sim_y)
            pos_m = triangulate(dA, dB, SENSOR_BASELINE_M)
        elif force_mouse or sensor.use_mouse:
            nx = max(0, min(1, (mx - field_rect.left) / field_rect.width))
            ny = max(0, min(1, (my - field_rect.top) / field_rect.height))
            pos_m = (nx * PLAY_AREA_W_M, ny * PLAY_AREA_D_M)
        else:
            pos_m = sensor.get_position_m(((mx - field_rect.left) / field_rect.width,
                                            (my - field_rect.top) / field_rect.height))

        cursor_px = None
        in_deadzone = False
        if pos_m is not None:
            px_norm = max(0, min(1, pos_m[0] / PLAY_AREA_W_M))
            py_norm = max(0, min(1, pos_m[1] / PLAY_AREA_D_M))
            cursor_px = (field_rect.left + px_norm * field_rect.width,
                         field_rect.top + py_norm * field_rect.height)
            in_deadzone = pos_m[1] < DEAD_ZONE_M

        # ---- Menu hover-and-dwell selection (no clicking) ----
        # Hover the hammer over a difficulty box; staying there for
        # DWELL_SECONDS winds the hammer back and then swings it down to
        # "hit" the option, which is what actually starts the game.
        # Moving away before the dwell completes cancels it.
        if not game_started and cursor_px is not None:
            hit_name = None
            for name in DIFFICULTY_ORDER:
                if boxes[name].collidepoint(cursor_px):
                    hit_name = name
                    break
            if exit_button_rect.collidepoint(cursor_px):
                hit_name = "EXIT"
            if hit_name != hover_target:
                hover_target = hit_name
                hover_elapsed = 0.0
            if hover_target is not None:
                hover_elapsed += dt
                if hover_elapsed >= DWELL_SECONDS:
                    if hover_target == "EXIT":
                        running = False
                    else:
                        difficulty = hover_target
                        game_started = True
                        score, level, time_left, spawn_timer = 0, 1, 60.0, 0.0
                        hammer_swing_timer = HAMMER_SWING_DURATION
                    hover_target, hover_elapsed = None, 0.0
        elif not game_started:
            hover_target, hover_elapsed = None, 0.0

        if game_started and cursor_px is not None and menu_button_rect.collidepoint(cursor_px):
            menu_button_hover_time += dt
            if menu_button_hover_time >= DWELL_SECONDS:
                game_started = False
                score, level, time_left, spawn_timer = 0, 1, 60.0, 0.0
                menu_button_hover_time = 0.0
        else:
            menu_button_hover_time = 0.0

        # ---- Game logic (only while playing) ----
        if game_started:
            time_left -= dt
            if time_left <= 0:
                game_started = False  # round over -> back to the menu grid

            spawn_timer -= dt
            if spawn_timer <= 0:
                idle = [m for m in moles if not m.up]
                if idle:
                    m = random.choice(idle)
                    preset = DIFFICULTY_PRESETS[difficulty]
                    duration = max(preset["min_dur"], preset["base_dur"] - level * preset["dur_decay"])
                    m.pop_up(duration)
                preset = DIFFICULTY_PRESETS[difficulty]
                spawn_timer = max(preset["min_spawn"], preset["base_spawn"] - level * preset["spawn_decay"])

            if cursor_px:
                for m in moles:
                    if m.contains(*cursor_px):
                        m.hit = True
                        score += 10
                        level = min(5, 1 + score // 50)
                        hammer_swing_timer = HAMMER_SWING_DURATION

        hammer_swing_timer = max(0.0, hammer_swing_timer - dt)

        for m in moles:
            m.update(dt)

        if screenshot_mode and frame_n == 45:
            pygame.image.save(screen, "screenshot_overlay.png" if overlay_screenshot else "screenshot.png")
            running = False

        # ---------------- DRAW ----------------
        logo_img = layout["logo_img"]
        bulb_count = layout["bulb_count"]
        bulb_gap = layout["bulb_gap"]
        bulb_start_x = layout["bulb_start_x"]
        scanline_overlay = layout["scanline_overlay"]

        screen.fill(COL_BG)

        cab_rect = pygame.Rect(10, 10, screen_w - 20, screen_h - 20)
        pygame.draw.rect(screen, COL_CAB, cab_rect, border_radius=6)
        pygame.draw.rect(screen, COL_MAZE_BLUE, cab_rect, width=4, border_radius=6)
        pygame.draw.rect(screen, COL_MAZE_BLUE, cab_rect.inflate(-10, -10), width=2, border_radius=6)
        for bx, by in [(cab_rect.left + 14, cab_rect.top + 14),
                       (cab_rect.right - 14, cab_rect.top + 14),
                       (cab_rect.left + 14, cab_rect.bottom - 14),
                       (cab_rect.right - 14, cab_rect.bottom - 14)]:
            pygame.draw.circle(screen, COL_PAC_YELLOW, (bx, by), 4)

        # Neon-blue maze bezel around the single 3x3 area
        bezel_rect = field_rect.inflate(16, 16)
        pygame.draw.rect(screen, COL_CAB_DK, bezel_rect, border_radius=8)
        pygame.draw.rect(screen, COL_MAZE_BLUE, bezel_rect, width=4, border_radius=8)
        pygame.draw.rect(screen, COL_MAZE_BLUE, bezel_rect.inflate(-10, -10), width=2, border_radius=8)
        pygame.draw.rect(screen, COL_SCREEN_BG, field_rect, border_radius=4)

        cell_w = field_rect.width / 3
        cell_h = field_rect.height / 3

        if not game_started:
            # ---------------- MENU: title row / difficulty row / help row ----------------
            top_rect = pygame.Rect(field_rect.left, field_rect.top, field_rect.width, cell_h)
            top_box = top_rect.inflate(-16, -16)
            pygame.draw.rect(screen, COL_CAB_DK, top_box, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, top_box, width=3, border_radius=6)
            # Marquee lights now run the FULL perimeter of the title box,
            # always chasing (not just a single row under the logo).
            draw_light_ring(screen, top_box, frame_n, count=28)

            if logo_img:
                screen.blit(logo_img, logo_img.get_rect(center=top_box.center))
            else:
                logo = font_pixel_lg.render("WACK A MOLE", True, COL_PAC_YELLOW)
                screen.blit(logo, logo.get_rect(center=top_box.center))

            pygame.draw.rect(screen, COL_CAB_DK, exit_button_rect, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, exit_button_rect, width=3, border_radius=6)
            if hover_target == "EXIT":
                draw_light_ring(screen, exit_button_rect.inflate(6, 6), frame_n)
            exit_label = font_pixel_xs.render("EXIT", True, COL_PAC_YELLOW)
            screen.blit(exit_label, exit_label.get_rect(center=exit_button_rect.center))

            for name in DIFFICULTY_ORDER:
                rect = boxes[name]
                is_hovered = (hover_target == name)
                pygame.draw.rect(screen, COL_CAB_DK, rect, border_radius=6)
                pygame.draw.rect(screen, COL_MAZE_BLUE, rect, width=3, border_radius=6)
                if is_hovered:
                    draw_light_ring(screen, rect.inflate(6, 6), frame_n)
                label = font_pixel_md.render(name.upper(), True, COL_PAC_YELLOW)
                screen.blit(label, label.get_rect(center=rect.center))

            # decorative empty holes either side of "help", matching the
            # gameplay grid's look so the menu still visually reads as the
            # same 3x3 cabinet
            for deco_name in ("deco_left", "deco_right"):
                rect = boxes[deco_name]
                mound_rect = pygame.Rect(0, 0, rect.width * 0.9, rect.height * 0.62)
                mound_rect.midbottom = (rect.centerx, rect.bottom - 6)
                pygame.draw.rect(screen, COL_MOUND_DARK, mound_rect, border_radius=int(mound_rect.height * 0.45))
                hole_rect = pygame.Rect(0, 0, mound_rect.width * 0.5, mound_rect.height * 0.3)
                hole_rect.center = (mound_rect.centerx, mound_rect.top + mound_rect.height * 0.3)
                pygame.draw.ellipse(screen, COL_HOLE, hole_rect)
                pygame.draw.ellipse(screen, COL_MAZE_BLUE, hole_rect, 1)

            help_rect = boxes["help"]
            help_hovered = cursor_px is not None and help_rect.collidepoint(cursor_px)
            if screenshot_mode and overlay_screenshot and not diff_hover_screenshot:
                help_hovered = True  # force it on for the demo screenshot
            pygame.draw.rect(screen, COL_CAB_DK, help_rect, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, help_rect, width=3, border_radius=6)
            if help_hovered:
                draw_light_ring(screen, help_rect.inflate(6, 6), frame_n)
            qmark = font_pixel_xs.render("?", True, COL_PAC_YELLOW)
            screen.blit(qmark, qmark.get_rect(center=(help_rect.centerx, help_rect.centery - 14)))
            help_label = font_pixel_sm.render("HELP", True, COL_PAC_YELLOW)
            screen.blit(help_label, help_label.get_rect(center=(help_rect.centerx, help_rect.bottom - 16)))

            if help_hovered:
                tip_w, tip_h = int(field_rect.width * 0.86), int(cell_h * 0.86)
                tip_rect = pygame.Rect(0, 0, tip_w, tip_h)
                tip_rect.center = (field_rect.centerx, field_rect.top + 2 * cell_h + cell_h / 2)
                tip_surf = pygame.Surface((tip_rect.width, tip_rect.height), pygame.SRCALPHA)
                tip_surf.fill((0, 0, 0, 235))
                screen.blit(tip_surf, tip_rect.topleft)
                pygame.draw.rect(screen, COL_PAC_YELLOW, tip_rect, width=2, border_radius=6)
                lines = [
                    "Move to control the cursor.",
                    "Whack moles as they pop up.",
                    "Pick a box above to start:",
                    "Easy, Medium, or Hard.",
                ]
                for i, line in enumerate(lines):
                    surf = font_led_sm.render(line, True, COL_CREAM)
                    screen.blit(surf, surf.get_rect(center=(tip_rect.centerx, tip_rect.top + 16 + i * 20)))

            # Hammer cursor — also active on the menu now, since hovering
            # IS how you select (no clicking). Winds back as it dwells.
            if cursor_px:
                cx, cy = int(cursor_px[0]), int(cursor_px[1])
                windup_progress = min(1.0, hover_elapsed / DWELL_SECONDS) if hover_target else 0.0
                swing_progress = 1 - (hammer_swing_timer / HAMMER_SWING_DURATION) if hammer_swing_timer > 0 else 0.0
                draw_hammer(screen, cx, cy, swing_progress, windup_progress=windup_progress)

        else:
            # ---------------- GAMEPLAY: the 9 mole cells ----------------
            score_panel = pygame.Rect(field_rect.left + 14, field_rect.top - 30, 150, 24)
            pygame.draw.rect(screen, COL_CAB_DK, score_panel, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, score_panel, width=2, border_radius=6)
            score_text = font_led_sm.render(f"SCORE {score:03d}", True, COL_CREAM)
            screen.blit(score_text, score_text.get_rect(midleft=(score_panel.left + 10, score_panel.centery)))

            diff_panel = pygame.Rect(field_rect.centerx - 110, field_rect.top - 30, 110, 24)
            pygame.draw.rect(screen, COL_CAB_DK, diff_panel, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, diff_panel, width=2, border_radius=6)
            diff_text = font_led_sm.render(f"DIFF {difficulty.upper()}", True, COL_CREAM)
            screen.blit(diff_text, diff_text.get_rect(midleft=(diff_panel.left + 10, diff_panel.centery)))

            time_panel = pygame.Rect(field_rect.right - 164, field_rect.top - 30, 150, 24)
            pygame.draw.rect(screen, COL_CAB_DK, time_panel, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, time_panel, width=2, border_radius=6)
            time_text = font_led_sm.render(f"TIME {time_left:05.1f}", True, COL_CREAM)
            screen.blit(time_text, time_text.get_rect(midleft=(time_panel.left + 10, time_panel.centery)))

            dot_positions = [
                (field_rect.left + field_rect.width * fx, field_rect.top + field_rect.height * fy)
                for fx in (0.17, 0.5, 0.83) for fy in (0.17, 0.5, 0.83)
            ]
            for dx, dy in dot_positions:
                pygame.draw.circle(screen, COL_DOT, (int(dx), int(dy - cell_h * 0.42)), 2)

            for m in moles:
                mound_rect = pygame.Rect(0, 0, m.rect.width, m.rect.height * 0.62)
                mound_rect.midbottom = (m.rect.centerx, m.rect.bottom)
                pygame.draw.rect(screen, COL_MOUND_DARK, mound_rect, border_radius=int(mound_rect.height * 0.45))
                highlight_rect = mound_rect.inflate(-mound_rect.width * 0.10, -mound_rect.height * 0.30)
                highlight_rect.top = mound_rect.top + 4
                pygame.draw.rect(screen, COL_MOUND, highlight_rect, border_radius=int(highlight_rect.height * 0.5))

                hole_rect = pygame.Rect(0, 0, m.rect.width * 0.5, m.rect.height * 0.22)
                hole_rect.center = (m.rect.centerx, mound_rect.top + mound_rect.height * 0.30)

                pygame.draw.ellipse(screen, COL_HOLE, hole_rect)
                pygame.draw.ellipse(screen, COL_MAZE_BLUE, hole_rect, 1)
                draw_mole(screen, hole_rect, m.pop_progress)

            pygame.draw.rect(screen, COL_CAB_DK, menu_button_rect, border_radius=6)
            pygame.draw.rect(screen, COL_MAZE_BLUE, menu_button_rect, width=3, border_radius=6)
            if cursor_px is not None and menu_button_rect.collidepoint(cursor_px):
                draw_light_ring(screen, menu_button_rect.inflate(6, 6), frame_n)
            menu_label = font_pixel_xs.render("MENU", True, COL_PAC_YELLOW)
            screen.blit(menu_label, menu_label.get_rect(center=menu_button_rect.center))

            if cursor_px:
                cx, cy = int(cursor_px[0]), int(cursor_px[1])
                swing_progress = 1 - (hammer_swing_timer / HAMMER_SWING_DURATION) if hammer_swing_timer > 0 else 0.0
                draw_hammer(screen, cx, cy, swing_progress, danger=in_deadzone)
            elif not sensor.use_mouse and not force_mouse and not sensor_demo_mode:
                msg = font_led_sm.render("NO RELIABLE SENSOR FIX - CHECK HARDWARE", True, COL_ALARM)
                screen.blit(msg, msg.get_rect(center=field_rect.center))

        screen.blit(scanline_overlay, field_rect.topleft)

        pygame.display.flip()

        if capture_gif_dir:
            pygame.image.save(screen, os.path.join(capture_gif_dir, f"f{frame_n:04d}.png"))
            if frame_n >= capture_frames:
                running = False

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    run()
