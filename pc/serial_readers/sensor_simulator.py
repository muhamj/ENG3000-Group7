"""
Simulated sensor input for game_wifi.py

Run this ALONGSIDE the game (as a second, separate process) to drive the
mole-hunting cabinet without real ESP32 hardware attached. It opens its
own small pygame window and sends UDP packets shaped exactly like the
ones the master ESP32 sends over Wi-Fi (mac/sensor1/sensor2/sensor3/
sensor4/closest), so game_wifi.py's existing UdpDistanceReader and
parse_udp_packet() pick them up with zero changes to game_wifi.py.

Sensor-to-board mapping (matches master.cpp / game_wifi.py):
    ESP1 (master)  -> sensor1 (left-facing), sensor2 (right-facing)
    ESP2 (slave)   -> sensor3 (left-facing), sensor4 (right-facing)
game_wifi.py's LEFT_SENSOR_NAMES=(sensor1,sensor3) and
RIGHT_SENSOR_NAMES=(sensor2,sensor4) confirm this: sensor1/sensor3 are
both "left" readings from two different boards, sensor2/sensor4 are
both "right" readings, and combine_left_right() takes whichever of
each side's two boards is closer.

This simulator now models ESP1 and ESP2 as two INDEPENDENT boxes, each
with its own left/right sensor pair, rather than just duplicating one
box's numbers onto the other. Two real, physically separate ultrasonic
sensors aimed at the same person will never report perfectly identical
distances, so each of the 4 readings is computed from its own board's
position plus a small amount of random jitter (SENSOR_NOISE_STD_CM).

ESP2's position is set via BOX2_X_OFFSET_CM / BOX2_DEPTH_OFFSET_CM
below, relative to ESP1 at x=0/depth=0. The defaults assume the two
boards are mounted right next to each other (0, 0) - if your actual
cabinet has ESP2 mounted further to one side or further forward/back
than ESP1, change those two constants to match and the 4 readings will
diverge accordingly, not just from noise but from real geometry.

The two programs don't need to be started in any particular order and
don't share any files - they only talk over a UDP socket on localhost,
so just run each in its own terminal:

    # terminal 1
    python game_wifi.py

    # terminal 2
    python sensor_simulator.py

Controls (in the simulator window):
    - Move the mouse: simulates a person moving left/right (x) and
      near/far (y) in front of the sensors. Top of the window = right
      up against the sensors, bottom = far away.
    - SPACE: toggle sending on/off ("nobody in front of sensors" - lets
      you test the game's SENSOR_TIMEOUT_MS fallback back to mouse
      control after ~500ms of silence).
    - N: toggle sensor noise on/off (useful if you want to sanity-check
      exact geometry without jitter in the way).
    - Numpad-style 1-9 keys: jump straight to a specific 3x3 grid cell
      (7 8 9 = top/far row, 4 5 6 = middle row, 1 2 3 = bottom/near row)
      and hold it there until you move the mouse again.
    - ESC / close window: quit.
"""

import json
import random
import socket

import pygame

UDP_IP = "127.0.0.1"
UDP_PORT = 4210          # must match UDP_PORT in game_wifi.py
SEND_HZ = 20              # packets per second

WINDOW_W, WINDOW_H = 500, 500
SENSOR_BASELINE_CM = 100.0   # distance between each board's own left and right sensor
MAX_DEPTH_CM = 200.0         # matches SENSOR_MAX_DISTANCE_CM in the game
EXTRA_MARGIN_CM = 30.0       # lets you "stand" a bit outside the sensor span
MIN_VALID_CM = 1.0           # keep readings > 0 so the game's parser accepts them
FAKE_MAC = "AA:BB:CC:DD:EE:FF"  # placeholder; game_wifi.py doesn't use this field

# ESP2's position relative to ESP1 (ESP1's left sensor is the x=0/depth=0
# origin). Tune these two to match your real cabinet - e.g. if ESP2 sits
# 15cm to the right of and 8cm further from the play area than ESP1:
#   BOX2_X_OFFSET_CM = 15.0
#   BOX2_DEPTH_OFFSET_CM = 8.0
BOX2_X_OFFSET_CM = 0.0
BOX2_DEPTH_OFFSET_CM = 0.0

# Small per-reading jitter so sensor1/sensor3 and sensor2/sensor4 aren't
# bit-for-bit identical even when the two boxes are co-located, like real
# ultrasonic sensors. Set to 0.0 to disable, or toggle live with 'N'.
SENSOR_NOISE_STD_CM = 1.5

# Fixed reference points used by the 1-9 grid-cell shortcut keys.
GRID_ROW_DEPTH_CM = {0: 170.0, 1: 75.0, 2: 20.0}   # far, mid, near
GRID_COL_X_FRAC = {0: 0.15, 1: 0.5, 2: 0.85}        # left, center, right
NUMPAD_TO_CELL = {
    pygame.K_7: 0, pygame.K_8: 1, pygame.K_9: 2,
    pygame.K_4: 3, pygame.K_5: 4, pygame.K_6: 5,
    pygame.K_1: 6, pygame.K_2: 7, pygame.K_3: 8,
}


def _distance_to_sensor(x_cm, depth_cm, sensor_x_cm, sensor_depth_cm, noise_std_cm):
    """Straight-line distance from a person at (x_cm, depth_cm) to one
    ultrasonic sensor at (sensor_x_cm, sensor_depth_cm), with optional
    Gaussian jitter to mimic real sensor noise."""
    dx = x_cm - sensor_x_cm
    dz = depth_cm - sensor_depth_cm
    distance = (dx ** 2 + dz ** 2) ** 0.5
    if noise_std_cm > 0:
        distance += random.gauss(0.0, noise_std_cm)
    return min(max(distance, MIN_VALID_CM), MAX_DEPTH_CM)


def window_pos_to_xy(mx, my):
    """Map a mouse position in the simulator window to a "true" world
    position in cm, independent of any one sensor's viewpoint."""
    x_frac = mx / WINDOW_W          # 0 = left edge, 1 = right edge
    depth_frac = my / WINDOW_H      # 0 = at the sensors, 1 = far away

    x_cm = -EXTRA_MARGIN_CM + x_frac * (SENSOR_BASELINE_CM + 2 * EXTRA_MARGIN_CM)
    depth_cm = 5.0 + depth_frac * (MAX_DEPTH_CM - 5.0)
    return x_cm, depth_cm


def grid_cell_to_xy(cell_index):
    row, col = divmod(cell_index, 3)
    depth_cm = GRID_ROW_DEPTH_CM[row]
    x_cm = GRID_COL_X_FRAC[col] * SENSOR_BASELINE_CM
    return x_cm, depth_cm


def compute_four_readings(x_cm, depth_cm, noise_enabled):
    """Independently compute all 4 sensor readings for a person at world
    position (x_cm, depth_cm): ESP1's left/right sensors at the origin,
    and ESP2's left/right sensors offset by BOX2_X_OFFSET_CM /
    BOX2_DEPTH_OFFSET_CM. Returns (sensor1, sensor2, sensor3, sensor4)."""
    noise_std = SENSOR_NOISE_STD_CM if noise_enabled else 0.0

    sensor1 = _distance_to_sensor(x_cm, depth_cm, 0.0, 0.0, noise_std)
    sensor2 = _distance_to_sensor(x_cm, depth_cm, SENSOR_BASELINE_CM, 0.0, noise_std)
    sensor3 = _distance_to_sensor(
        x_cm, depth_cm, BOX2_X_OFFSET_CM, BOX2_DEPTH_OFFSET_CM, noise_std
    )
    sensor4 = _distance_to_sensor(
        x_cm, depth_cm,
        BOX2_X_OFFSET_CM + SENSOR_BASELINE_CM, BOX2_DEPTH_OFFSET_CM,
        noise_std,
    )
    return sensor1, sensor2, sensor3, sensor4


def build_packet(sensor1, sensor2, sensor3, sensor4):
    """Shape the packet exactly like the master ESP32's UDP send in
    master.cpp: mac/sensor1/sensor2/sensor3/sensor4/closest."""
    closest = min(sensor1, sensor2, sensor3, sensor4)
    return json.dumps({
        "mac": FAKE_MAC,
        "sensor1": round(sensor1, 2),
        "sensor2": round(sensor2, 2),
        "sensor3": round(sensor3, 2),
        "sensor4": round(sensor4, 2),
        "closest": round(closest, 2),
    })


def main():
    pygame.init()
    pygame.display.set_caption("Sensor simulator (feeds game_wifi.py)")
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    mx, my = WINDOW_W // 2, WINDOW_H // 2
    x_cm, depth_cm = window_pos_to_xy(mx, my)
    sending = True
    noise_enabled = True

    pygame.mouse.get_rel()  # zero out the relative-motion counter before the loop

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    sending = not sending
                elif event.key == pygame.K_n:
                    noise_enabled = not noise_enabled
                elif event.key in NUMPAD_TO_CELL:
                    x_cm, depth_cm = grid_cell_to_xy(NUMPAD_TO_CELL[event.key])

        # Only re-derive position from the mouse if it actually moved this
        # frame, so a numpad shortcut "sticks" until you move the mouse again.
        rel_x, rel_y = pygame.mouse.get_rel()
        if pygame.mouse.get_focused() and (rel_x != 0 or rel_y != 0):
            mx, my = pygame.mouse.get_pos()
            mx = min(max(mx, 0), WINDOW_W)
            my = min(max(my, 0), WINDOW_H)
            x_cm, depth_cm = window_pos_to_xy(mx, my)

        # Recompute all 4 readings every frame (not just on movement) so
        # noise varies over time even while standing still, like real
        # ultrasonic sensors do.
        sensor1, sensor2, sensor3, sensor4 = compute_four_readings(x_cm, depth_cm, noise_enabled)

        if sending:
            packet = build_packet(sensor1, sensor2, sensor3, sensor4)
            sock.sendto(packet.encode("utf-8"), (UDP_IP, UDP_PORT))
            # Print the exact bytes handed to game_wifi.py - not a
            # reconstructed summary - so the terminal always matches what
            # parse_udp_packet() on the other end actually receives.
            print(f"-> {UDP_IP}:{UDP_PORT}  {packet}")

        screen.fill((30, 30, 30))
        status_line1 = (
            f"{'SENDING' if sending else 'PAUSED (space to resume)'}  "
            f"noise={'on' if noise_enabled else 'off'} (N to toggle)"
        )
        status_line2 = (
            f"ESP1: sensor1={sensor1:.1f}cm  sensor2={sensor2:.1f}cm   "
            f"ESP2: sensor3={sensor3:.1f}cm  sensor4={sensor4:.1f}cm"
        )
        screen.blit(font.render(status_line1, True, (255, 255, 255)), (10, 10))
        screen.blit(font.render(status_line2, True, (255, 255, 255)), (10, 34))
        screen.blit(
            font.render(
                "Mouse = position | 1-9 = jump to grid cell | Space = pause",
                True, (200, 200, 200),
            ),
            (10, WINDOW_H - 30),
        )
        pygame.draw.circle(screen, (255, 80, 80), (int(mx), int(my)), 8)
        pygame.display.flip()
        clock.tick(SEND_HZ)

    sock.close()
    pygame.quit()


if __name__ == "__main__":
    main()