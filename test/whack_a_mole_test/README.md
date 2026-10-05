# Whack-a-Mole — Arcade Cabinet

`whack_a_mole_game.py` is the Pygame version of the black arcade design
(`whack_a_mole_ui_redesign.html`). It has the same look and the same 3x3
layout, plus the score tracker, 30-mole rounds and the Game Over popup.
All sensor handling is the same as the team's working game file, and it
falls back to the mouse when no sensor data is arriving.

## Folder layout

Keep these together:

```
whack_a_mole/
├── whack_a_mole_game.py
└── assets/
    ├── logo.png
    └── fonts/
        ├── PressStart2P.ttf
        └── VT323.ttf
```

## Running

```bash
pip install pygame
python whack_a_mole_game.py
```

Option: `--scale 1.3` for a bigger window (or `--scale 0.9` for a smaller one).

## Using the sensors

The sensor handling is the same as the team's working game file:

1. Start `python sensor_monitor.py` **before** the game. The game connects
   to `http://localhost:8000/data` once at startup and prints whether it
   found it.
2. If the firmware sends `x`/`y`, those are mapped to the grid: 40 cm side
   columns on a 150 cm baseline, rows at 55–100 / 100–140 / 140–180 cm,
   closest row at the **bottom**, and anything nearer than 55 cm is a dead
   zone (no cell).
3. Otherwise the four raw readings are combined left/right as before.
4. If no new sensor frame arrives for 0.5 s, the mouse takes over.

Each sensor frame is printed to the console (`Sensor position: cell=...`).

## Controls

- **Menu:** stay on Easy, Medium or Hard for 2 seconds to start. Hover
  HELP for instructions.
- **Game:** move onto a mole's square; the hit lands after 50 ms (+10).
- After 30 moles, the Game Over popup shows your result and returns to
  the menu after 5 seconds.
- `ESC` goes back to the menu, or quits from the menu. The MENU button
  (top right) also works with the mouse.

## Settings

At the top of `whack_a_mole_game.py`: `ROUND_MOLES`, `POINTS_PER_HIT`,
`RESULTS_MS`, `DIFFICULTY_PRESETS` and `UI_SCALE`. The sensor settings
(distances, `HAMMER_WIND_MS`, `MENU_CONFIRM_MS`) are in their own block
below, with the same values as the team's file.
