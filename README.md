# Full-Body Whack-a-Mole

A full-body, motion-controlled take on the classic arcade game. Players move within a tracked play area to "whack" randomly appearing moles on screen and no controllers, no cameras, just ultrasonic sensing.


## Overview

Instead of a handheld controller, the player's on-screen cursor is driven by their body position in a physical play area. Ultrasonic sensors track the player's location and stream that data wirelessly to a PC, which converts it into cursor movement and drives the game logic.

## Features

- **Full-body motion control** — cursor position mapped from real-world player movement
- **Ultrasonic-only tracking** — no cameras or optical sensors involved
- **Wireless sensor network** — sensor units communicate to the PC over [Wi-Fi/Bluetooth — *update*]
- **Progressive difficulty** — multiple levels with increasing mole speed / spawn rate / target size
- **Dead-zone safety system** — audible alarm + visual warning if the player gets too close to the screen
- **Battery-powered hardware** — standalone sensor units run on AA NiMH batteries, no wall power
- **Windows-installable build** — packaged as a standalone application, no dev environment needed to run

## How It Works

1. Ultrasonic sensors (mounted near the screen) continuously measure distance to the player
2. Each sensor unit reads locally and transmits data wirelessly to the host PC
3. The PC combines readings from multiple sensors to calculate the player's (x, y) position within the play area
4. That position drives an on-screen cursor
5. The game spawns moles at random positions; moving the cursor onto a mole before it disappears scores a point
6. If the calculated position enters the dead zone near the screen, the system triggers an audible + visual warning

## Hardware

| Component | Purpose |
|---|---|
| [ESP32| Microcontroller per sensor unit, handles sensing + wireless transmission |
| [RCWL-1601 ultrasonic sensor — *update*] | Distance/position sensing |
| 4x AA NiMH battery pack | Standalone power per sensor unit (1hr+ runtime) |
| [Perf board / enclosure — *update*] | Housing per sensor unit (≤100×100×50mm) |


## Project Structure

/firmware       # Microcontroller code for sensor units
/game           # PC-side game application
/docs           # Diagrams, calibration notes, demo media

---

# Basic Understanding of the Wack a Mole game
----------------------------------------------
## The following components can be used (and will be supplied):
- 2 x ESP32 processor board ($8 each)
- 2 x Antennas ($0 each)
- 4 x Ultrasonic sensor – RCWL-1601 ($5 each)
- 2 x Battery Pack (each pack incl. 4 AA NiMH batteries and a Battery holder)
($7 each - discounted price)
- Perf board
- Connectors
 
## Eqiupment Use
### LED
- LED/ Displays going to be on the wall
  * Display and User interface is to be done by SE (but check in with electrical for placement and sensor understanding)
### Sensors
- Push buttons (hopefully)
- Ultra sonic
- if not using normal sensors and covering hand over it to select it
### Software
- Aim to use Platform IO
- Python + PY Game?
- Arduino (possibly no ESP 32)
----------------------------------------------
# The assessment of the Whack-a-Mole system will be based on the following criteria:
## 1. Full Functionality
   - Sends sensor data wirelessly to the PC
   - Runs on and recharges supplied batteries
   - Integrates sensor data into calibrated spatial positioning
   - Alarm when within 60cm of screen
   - Original Whack-a-Mole game with levels
   - Play the Whack-a-Mole game based on sensor data
## 2. Performance Benchmarks
   - Sensors less than 100 mm x 100 mm x 50 mm
   - Current consumption and battery data show life > 1 hour
   - Person tracking is suƯiciently accurate to play game
   - Person tracking is suƯiciently responsive to play game
## 3. Design and Manufacture
   - Code can be downloaded and installed on a Windows laptop.
   - Sensors have neat external appearance
   - Sensors have sound electronic construction
   - Supplied components are returned in good working order
   - No solder, adhesives, paint or other like material has been applied to the supplied components 
# Functionality
