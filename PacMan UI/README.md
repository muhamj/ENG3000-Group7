# Whack-a-Mole — Arcade UI/UX Redesign

Two things live in this folder:

1. **`whack_a_mole_game.py`** — a real, runnable Pygame build, wired to
   read actual ultrasonic sensor data (with a mouse fallback for testing
   without hardware).
2. **`whack_a_mole_ui_redesign.html`** — a browser mockup of the same
   design. Double-click to open in any browser.

## Running the Python game

```bash
pip install pygame pyserial
python whack_a_mole_game.py
```

Keep this folder structure as-is:

```
whack_a_mole/
├── whack_a_mole_game.py
├── whack_a_mole_ui_redesign.html
└── assets/
    ├── logo.png
    └── fonts/
        ├── PressStart2P.ttf
        └── VT323.ttf
```

## How it works

The whole game — title, difficulty picker, help, and the mole-whacking
grid — lives inside a single 3x3 area. No separate score/level/time/
dead-zone bars.

- **Move the mouse** (or your real ultrasonic sensors, once wired up) to
  control the hammer cursor.
- **Hover a difficulty box (Easy / Medium / Hard) for 2 seconds** to
  select it — no clicking. The hammer visibly winds back the whole time
  you're hovering, then swings down to "hit" the option once the dwell
  completes, which is what actually starts the game. Moving away early
  cancels it.
- **Hover "?" HELP** to see instructions.
- Chase lights run the full perimeter of the title banner at all times,
  and light up around a difficulty box's perimeter while you're hovering
  it (charging toward selection).
- Whack moles as they pop up during play. Press `M` to toggle mouse vs.
  live sensor control, or use a small "MENU" button (top corner during
  play) to go back and pick a different difficulty.

## Wiring up your real ESP32 sensors

1. **`SERIAL_PORT`** near the top of `whack_a_mole_game.py` — set this to
   your ESP32's actual COM port / device path.
2. **`parse_serial_line()`** — assumes lines like `S1: Distance: 9.40 cm`.
   If your team's format differs, this is the one function to edit.

If no serial device is found, the game automatically falls back to mouse
control instead of crashing.
