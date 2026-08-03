PlatformIO project scaffold

Quick steps to get started

1. Install the PlatformIO extension in VS Code: open Extensions (Ctrl+Shift+X), search for "PlatformIO" and install.
2. Open this workspace and use the PlatformIO commands from the Command Palette (Ctrl+Shift+P):
   - `PlatformIO: Home`
   - `PlatformIO: Build`
   - `PlatformIO: Upload`
3. Change the board in `platformio.ini` if you are not using an Arduino Uno. See the commented example for ESP32.
4. If you prefer the CLI, install PlatformIO Core:

```
python -m pip install -U platformio
pio run      # build
pio run -t upload  # upload (when a board/port is configured)
```

If you tell me your target board, I can add a matching environment to `platformio.ini`.
