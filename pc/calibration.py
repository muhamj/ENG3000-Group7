"""Standalone floor-grid sensor calibration recorder.

Run sensor_monitor.py first, then run this file with Python. The output is
JSON containing every fresh sensor frame grouped by its physical grid square.
"""

import json
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox
from datetime import datetime, timezone

import pygame

try:
    from game_wifi import SENSOR_MONITOR_DATA_URL, SensorMonitorReader
except ImportError as exc:
    raise SystemExit("Run this script from pc/ and install pygame first.") from exc


WIDTH, HEIGHT = 900, 700
BG = (18, 23, 32)
PANEL = (30, 39, 52)
TEXT = (239, 243, 248)
MUTED = (156, 171, 190)
BLUE = (76, 155, 255)
GREEN = (80, 197, 139)
AMBER = (244, 185, 66)
RED = (232, 93, 93)
HEAT_COLD = (52, 116, 210)
HEAT_HOT = (232, 83, 63)


def make_session(rows, columns, duration):
    cells = []
    for row in range(rows):
        for column in range(columns):
            cells.append({"row": row + 1, "column": column + 1, "samples": []})
    return {
        "format": "floor-grid-calibration-v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "grid": {"rows": rows, "columns": columns},
        "duration_per_square_seconds": duration,
        "coordinate_note": "row 1 is the top row; column 1 is the left column",
        "cells": cells,
    }


def main():
    setup = tk.Tk()
    setup.title("Floor Grid Calibration Setup")
    setup.resizable(False, False)
    setup.configure(bg="#121720")
    setup.geometry("380x320")
    fields = {}
    for label, key, value in (("Grid rows", "rows", "3"), ("Grid columns", "columns", "3"), ("Seconds per square", "seconds", "5")):
        row = tk.Frame(setup, bg="#121720", padx=24, pady=10)
        row.pack(fill="x")
        tk.Label(row, text=label, fg="#eff3f8", bg="#121720", font=("Segoe UI", 11)).pack(side="left")
        entry = tk.Entry(row, width=8, font=("Segoe UI", 11))
        entry.insert(0, value)
        entry.pack(side="right")
        fields[key] = entry
    tk.Label(setup, text="Each square records automatically, then a short move countdown starts.",
             wraplength=330, fg="#9cabbE", bg="#121720", font=("Segoe UI", 9)).pack(pady=10)
    settings = {}

    def begin():
        try:
            settings["rows"] = int(fields["rows"].get())
            settings["columns"] = int(fields["columns"].get())
            settings["seconds"] = float(fields["seconds"].get())
            if not (1 <= settings["rows"] <= 12 and 1 <= settings["columns"] <= 12 and 0.1 <= settings["seconds"] <= 3600):
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid settings", "Rows and columns must be 1–12; seconds must be 0.1–3600.", parent=setup)
            return
        setup.destroy()

    tk.Button(setup, text="Start calibration", command=begin, bg="#4c9bff", fg="white",
              activebackground="#367fd8", relief="flat", padx=18, pady=9,
              font=("Segoe UI", 11, "bold")).pack(pady=6)
    setup.mainloop()
    if not settings:
        return 0
    rows, columns, duration = settings["rows"], settings["columns"], settings["seconds"]
    output = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "calibration_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".json",
    )

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Floor Grid Calibration")
    font = pygame.font.SysFont("segoeui", 25)
    small = pygame.font.SysFont("segoeui", 18)
    clock = pygame.time.Clock()
    reader = SensorMonitorReader()
    session = make_session(rows, columns, duration)
    active_index = 0
    state = "countdown"
    countdown_ends_at = time.monotonic() + 3
    started_at = None
    elapsed_before_pause = 0.0
    last_sample_at = None
    error = None
    saved = False
    imported = False
    import_button = pygame.Rect(735, 45, 125, 38)

    def load_json():
        nonlocal session, rows, columns, duration, active_index, state, imported, saved
        picker = tk.Tk()
        picker.withdraw()
        path = filedialog.askopenfilename(
            parent=picker, title="Import calibration readings",
            filetypes=(("Calibration JSON", "*.json"), ("All files", "*.*")),
        )
        if not path:
            picker.destroy()
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                candidate = json.load(handle)
            grid = candidate["grid"]
            new_rows, new_columns = int(grid["rows"]), int(grid["columns"])
            cells = candidate["cells"]
            if not (1 <= new_rows <= 12 and 1 <= new_columns <= 12) or not isinstance(cells, list):
                raise ValueError("invalid grid dimensions or cells list")
            if len(cells) != new_rows * new_columns:
                raise ValueError("the JSON must contain one cell entry for every grid square")
            for index, cell in enumerate(cells):
                if not isinstance(cell, dict) or not isinstance(cell.get("samples"), list):
                    raise ValueError(f"square {index + 1} has no samples list")
                cell.setdefault("row", index // new_columns + 1)
                cell.setdefault("column", index % new_columns + 1)
            session = candidate
            rows, columns = new_rows, new_columns
            duration = float(candidate.get("duration_per_square_seconds", duration))
            active_index = 0
            state, imported, saved = "imported", True, True
        except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            messagebox.showerror("Import failed", f"Could not load calibration JSON:\n{exc}", parent=picker)
        finally:
            picker.destroy()

    def save():
        session["finished_at"] = datetime.now(timezone.utc).isoformat()
        session["completed_squares"] = sum(bool(cell["samples"]) for cell in session["cells"])
        session["sample_count"] = sum(len(cell["samples"]) for cell in session["cells"])
        target = os.path.abspath(output)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as handle:
            json.dump(session, handle, indent=2)
            handle.write("\n")
        return target

    def draw_text(value, x, y, color=TEXT, selected_font=font):
        screen.blit(selected_font.render(value, True, color), (x, y))

    running = True
    while running:
        now = time.monotonic()
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and import_button.collidepoint(event.pos):
                load_json()
            if event.type == pygame.QUIT:
                if not saved:
                    try:
                        saved_path = save()
                        error = f"Stopped; partial session saved to {saved_path}"
                        saved = True
                    except OSError as exc:
                        error = f"Could not save session: {exc}"
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    if not saved:
                        try:
                            saved_path = save()
                            error = f"Stopped; partial session saved to {saved_path}"
                            saved = True
                        except OSError as exc:
                            error = f"Could not save session: {exc}"
                    running = False
                elif event.key == pygame.K_SPACE and state in ("countdown", "recording", "move_countdown", "paused"):
                    if state == "paused":
                        state = paused_state[0]
                        if state in ("countdown", "move_countdown"):
                            countdown_ends_at = now + paused_countdown[0]
                        else:
                            started_at = now - elapsed_before_pause
                    else:
                        paused_state = [state]
                        paused_countdown = [max(0, countdown_ends_at - now) if state in ("countdown", "move_countdown") else 0]
                        elapsed_before_pause = (now - started_at) if state == "recording" else 0
                        state = "paused"
                elif event.key == pygame.K_s and state != "stopped":
                    try:
                        saved_path = save()
                        saved = True
                        state = "stopped"
                        error = f"Session saved to {saved_path}"
                    except OSError as exc:
                        error = f"Could not save session: {exc}"

        if state == "countdown" and now >= countdown_ends_at:
            state, started_at = "recording", now
        elif state == "move_countdown" and now >= countdown_ends_at:
            active_index += 1
            if active_index >= len(session["cells"]):
                try:
                    saved_path = save()
                    saved = True
                    state = "complete"
                    error = f"All squares recorded. Saved to {saved_path}"
                except OSError as exc:
                    state, error = "stopped", f"Could not save session: {exc}"
            else:
                state, countdown_ends_at = "countdown", now + 3

        if state == "recording":
            try:
                reading = reader.read_distances()
                if reading is not None:
                    cell = session["cells"][active_index]
                    sample = {
                        "elapsed_seconds": round(now - started_at, 3),
                        "sensor1_cm": reading.get("sensor1"),
                        "sensor2_cm": reading.get("sensor2"),
                        "sensor3_cm": reading.get("sensor3"),
                        "sensor4_cm": reading.get("sensor4"),
                        "x": reading.get("x"),
                        "y": reading.get("y"),
                    }
                    cell["samples"].append(sample)
                    last_sample_at = now
                    error = None
                elapsed = now - started_at
                if elapsed >= duration:
                    countdown_ends_at = now + 3
                    state = "move_countdown"
            except Exception as exc:
                error = f"Recording error: {exc}"
                elapsed_before_pause = now - started_at
                paused_state = ["recording"]
                paused_countdown = [0]
                state = "paused"

        screen.fill(BG)
        pygame.draw.rect(screen, PANEL, (28, 28, WIDTH - 56, HEIGHT - 56), border_radius=16)
        draw_text("FLOOR GRID CALIBRATION", 58, 48)
        draw_text(f"Grid: {rows} rows × {columns} columns    |    {duration:g} sec per square", 58, 91, MUTED, small)
        pygame.draw.rect(screen, BLUE, import_button, border_radius=7)
        import_label = small.render("Import JSON", True, TEXT)
        screen.blit(import_label, import_label.get_rect(center=import_button.center))
        margin, top, gap = 58, 157, 8
        grid_w, grid_h = 315, 315
        cell_w = (grid_w - gap * (columns - 1)) / columns
        cell_h = (grid_h - gap * (rows - 1)) / rows
        for index, cell in enumerate(session["cells"]):
            row, col = divmod(index, columns)
            rect = pygame.Rect(margin + col * (cell_w + gap), top + row * (cell_h + gap), cell_w, cell_h)
            valid_ranges = []
            for sample in cell["samples"]:
                readings = [sample.get(f"sensor{i}_cm") for i in range(1, 5)]
                readings = [float(value) for value in readings if isinstance(value, (int, float)) and 0 < value <= 200]
                if readings:
                    valid_ranges.append(min(readings))
            average_range = sum(valid_ranges) / len(valid_ranges) if valid_ranges else None
            if average_range is None:
                fill = PANEL
            else:
                heat = max(0.0, min(1.0, 1 - average_range / 200))
                fill = tuple(round(cold + (hot - cold) * heat) for cold, hot in zip(HEAT_COLD, HEAT_HOT))
            if index == active_index and state not in ("complete", "stopped"):
                pygame.draw.rect(screen, fill, rect, border_radius=8)
                pygame.draw.rect(screen, TEXT, rect, width=3, border_radius=8)
            else:
                pygame.draw.rect(screen, fill, rect, border_radius=8)
            n = small.render(f"R{cell['row']} C{cell['column']}", True, TEXT)
            screen.blit(n, n.get_rect(center=rect.center))
            if cell["samples"]:
                label = f"{average_range:.0f} cm avg" if average_range is not None else f"{len(cell['samples'])} samples"
                count = small.render(label, True, TEXT)
                screen.blit(count, count.get_rect(center=(rect.centerx, rect.centery + 21)))

        progress = (active_index / len(session["cells"])) if session["cells"] else 1
        pygame.draw.rect(screen, (54, 65, 81), (58, 498, grid_w, 10), border_radius=5)
        pygame.draw.rect(screen, GREEN, (58, 498, int(grid_w * progress), 10), border_radius=5)
        current = session["cells"][min(active_index, len(session["cells"]) - 1)]
        info_x = 414
        completed = sum(bool(cell["samples"]) for cell in session["cells"])
        draw_text(f"Progress: {completed}/{len(session['cells'])} squares", info_x, 160, GREEN, small)
        if imported:
            draw_text("Imported readings · heat = nearer sensor range", info_x, 188, MUTED, small)
        else:
            draw_text("Heat map: average nearest sensor range per square", info_x, 188, MUTED, small)
        if state not in ("complete", "stopped", "imported"):
            draw_text(f"Stand in square R{current['row']} C{current['column']}", info_x, 202)
        if state == "recording":
            elapsed = now - started_at
            remaining = max(0, duration - elapsed)
            draw_text("RECORDING", info_x, 252, BLUE)
            draw_text(f"{remaining:.1f} seconds remaining", info_x, 290)
            draw_text(f"Samples collected: {len(current['samples'])}", info_x, 330, MUTED, small)
            if last_sample_at is None or now - last_sample_at > 2:
                draw_text("Waiting for sensor readings…", info_x, 366, AMBER, small)
            pygame.draw.rect(screen, (54, 65, 81), (info_x, 365, 380, 18), border_radius=8)
            pygame.draw.rect(screen, BLUE, (info_x, 365, int(380 * max(0, min(1, remaining / duration))), 18), border_radius=8)
        elif state in ("countdown", "move_countdown"):
            remaining = max(0, countdown_ends_at - now)
            draw_text("GET READY — MOVE TO THIS SQUARE" if state == "countdown" else "MOVE TO THE NEXT SQUARE", info_x, 252, AMBER)
            draw_text(f"{remaining:.1f}", info_x + 130, 295, TEXT)
            pygame.draw.rect(screen, (54, 65, 81), (info_x, 365, 380, 18), border_radius=8)
            pygame.draw.rect(screen, AMBER, (info_x, 365, int(380 * max(0, min(1, remaining / 3))), 18), border_radius=8)
        elif state == "paused":
            draw_text("PAUSED", info_x, 252, AMBER)
            draw_text("Fix any sensor issue, then press Space to resume", info_x, 290, TEXT, small)
        elif state == "complete":
            draw_text("CALIBRATION COMPLETE", info_x, 252, GREEN)
        elif state == "imported":
            draw_text("IMPORTED HEAT MAP", info_x, 252, GREEN)
            draw_text("Cool = farther reading   •   Warm = nearer reading", info_x, 290, TEXT, small)
        elif state == "stopped":
            draw_text("SESSION STOPPED", info_x, 252, AMBER)
        if error:
            draw_text(error[:65], info_x, 398, RED if "error:" in error.lower() or "could not" in error.lower() else MUTED, small)
        draw_text("Space: pause / resume     S: stop & save     Esc: save & exit", 58, 563, MUTED, small)
        draw_text("Samples include raw sensor ranges and firmware x/y coordinates when available.", 58, 590, MUTED, small)
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    reader.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
