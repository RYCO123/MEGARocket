"""
vision/calibrate_lens.py

Collects the pixel-to-angle calibration table used by CalibratedProjector.

Setup:
  - Mount the camera at a known, measured height pointing straight down
    (0 pitch, 0 roll)
  - Lay a ruler or tape measure on the ground directly under the camera

The camera view is rotated so the display matches the craft frame:
    +y (forward/nose) = UP on screen
    +x (right/stbd)   = RIGHT on screen

The script walks through every tick one at a time. The active tick is
highlighted in cyan; completed ticks turn green. Type the real-world
distance (cm) from center to that tick and press Enter — video keeps
running while you measure.

Usage:
    python3 vision/calibrate_lens.py
    python3 vision/calibrate_lens.py --source 0 --step 25
"""

import argparse
import csv
import math
import platform
import queue
import sys
import threading
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

RESULTS_DIR      = Path(__file__).resolve().parent / "results"
DEFAULT_CSV      = RESULTS_DIR / "calibration.csv"
DEFAULT_HEIGHT_CM = 80.0
DEFAULT_STEP     = 50
MARGIN_FRAC      = 0.90

# Colors (BGR)
C_CROSSHAIR = (100, 100, 100)
C_CENTER    = (0,   0,   220)   # red  — center + mark
C_ACTIVE    = (0,  230, 255)    # cyan — active tick
C_DONE      = (0,  180,  60)    # green
C_PENDING   = (50,  50,  50)    # dark gray
C_AXIS_X    = (60,  80, 255)    # red-orange — +x (right/stbd)
C_AXIS_Y    = (60, 220,  60)    # green      — +y (fwd/nose)
AXIS_LEN    = 60


# ── Drawing (operates on the already-rotated display frame) ──────────────────
#
# After 90° CCW rotation of the raw camera frame:
#   +x (right/stbd)  = image RIGHT  in display
#   +y (fwd/nose)    = image UP     in display
#
# Tick positions in the rotated frame:
#   axis='x' calibrates image-U direction (craft forward = UP in display)
#     → horizontal lines at  y_rot = cy_rot - tick
#   axis='y' calibrates image-V direction (craft right = RIGHT in display)
#     → vertical lines at    x_rot = cx_rot + tick

def draw_frame(rot_frame, axis, ticks, current_idx):
    """Annotate an already-rotated display frame for the current tick."""
    out = rot_frame.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2

    # Dim crosshair
    cv2.line(out, (0, cy), (w, cy), C_CROSSHAIR, 1)
    cv2.line(out, (cx, 0), (cx, h), C_CROSSHAIR, 1)

    # Red center +
    r = 12
    cv2.line(out, (cx - r, cy), (cx + r, cy), C_CENTER, 2)
    cv2.line(out, (cx, cy - r), (cx, cy + r), C_CENTER, 2)

    # Tick lines
    for i, tick in enumerate(ticks):
        if i < current_idx:
            color, thick, full = C_DONE, 1, False
        elif i == current_idx:
            color, thick, full = C_ACTIVE, 2, True
        else:
            color, thick, full = C_PENDING, 1, False

        if axis == 'x':
            # Craft forward = UP in display → horizontal line above center
            y = cy - tick
            if full:
                cv2.line(out, (0, y), (w, y), color, thick)
                cv2.putText(out, f"+{tick}px", (cx + 8, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            else:
                cv2.line(out, (cx - 40, y), (cx + 40, y), color, thick)
        else:
            # Craft right = RIGHT in display → vertical line right of center
            x = cx + tick
            if full:
                cv2.line(out, (x, 0), (x, h), color, thick)
                cv2.putText(out, f"+{tick}px", (x + 4, cy - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            else:
                cv2.line(out, (x, cy - 40), (x, cy + 40), color, thick)

    # Axis indicator — top-left corner (+x right, +y up)
    ox, oy = 68, 95
    cv2.arrowedLine(out, (ox, oy), (ox + AXIS_LEN, oy), C_AXIS_X, 2, tipLength=0.2)
    cv2.arrowedLine(out, (ox, oy), (ox, oy - AXIS_LEN), C_AXIS_Y, 2, tipLength=0.2)
    cv2.putText(out, "+x (right/stbd)", (ox + AXIS_LEN + 4, oy + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_AXIS_X, 1)
    cv2.putText(out, "+y (fwd/nose)", (ox + 4, oy - AXIS_LEN - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_AXIS_Y, 1)

    # Status bar
    tick_now = ticks[current_idx]
    if axis == 'x':
        direction = "+Y fwd/nose (ruler toward nose, tick line goes UP)"
    else:
        direction = "+X right/stbd (ruler toward starboard, tick line goes RIGHT)"
    cv2.putText(out,
                f"{direction}   +{tick_now}px   {current_idx+1}/{len(ticks)}"
                "   | type cm in terminal |",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, C_ACTIVE, 1)

    return out


# ── Camera helpers ────────────────────────────────────────────────────────────

def open_capture(source):
    backend = (cv2.CAP_AVFOUNDATION
               if isinstance(source, int) and platform.system() == 'Darwin'
               else cv2.CAP_ANY)
    cap = cv2.VideoCapture(source, backend)
    if hasattr(cv2, 'CAP_PROP_BUFFERSIZE'):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_frame(cap):
    """Flush buffer and return latest frame."""
    frame = None
    for _ in range(4):
        ret, f = cap.read()
        if ret:
            frame = f
    return frame


# ── Background stdin reader ───────────────────────────────────────────────────

def _stdin_reader(q):
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        q.put(line.strip())


# ── Main calibration loop ─────────────────────────────────────────────────────

def run(cap, height_cm: float, step: int, csv_path: Path):
    # Wait for first frame to know resolution
    frame = None
    print("Waiting for camera…")
    cv2.namedWindow("Calibration", cv2.WINDOW_NORMAL)
    while frame is None:
        frame = read_frame(cap)
        cv2.waitKey(30)

    h_orig, w_orig = frame.shape[:2]
    # Ticks are in original-frame pixel units
    x_ticks = list(range(step, int((w_orig // 2) * MARGIN_FRAC) + 1, step))
    y_ticks = list(range(step, int((h_orig // 2) * MARGIN_FRAC) + 1, step))

    print(f"Original frame: {w_orig}×{h_orig}")
    print(f"Fwd/nose ticks (+Y, image-X axis): {x_ticks}")
    print(f"Right/stbd ticks (+X, image-Y axis): {y_ticks}")
    print(f"Camera height: {height_cm} cm\n")

    # Background thread reads terminal input; main loop runs video
    input_q: queue.Queue = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(input_q,), daemon=True).start()

    records = []

    for axis, ticks in [('x', x_ticks), ('y', y_ticks)]:
        if axis == 'x':
            print("─── Calibrating +Y direction (fwd/nose) ───")
            print("Orient ruler toward the nose. Tick lines appear ABOVE center.\n")
        else:
            print("─── Calibrating +X direction (right/stbd) ───")
            print("Orient ruler toward starboard. Tick lines appear RIGHT of center.\n")

        for idx, tick in enumerate(ticks):
            print(f"  +{tick:4d}px  →  distance (cm): ", end='', flush=True)

            dist_cm = None
            while dist_cm is None:
                # Keep video live
                fresh = read_frame(cap)
                if fresh is not None:
                    frame = fresh
                rot = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow("Calibration", draw_frame(rot, axis, ticks, idx))
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    print("\nAborted.")
                    cv2.destroyAllWindows()
                    return

                try:
                    raw = input_q.get_nowait()
                    if not raw:
                        continue
                    try:
                        val = float(raw)
                    except ValueError:
                        print(f"\n    Not a number — try again: ", end='', flush=True)
                        continue
                    if val <= 0:
                        print(f"\n    Must be > 0 — try again: ", end='', flush=True)
                        continue
                    dist_cm = val
                except queue.Empty:
                    pass

            theta = math.atan2(dist_cm, height_cm)
            print(f"{dist_cm}  → θ={math.degrees(theta):.3f}°")
            records.append({
                'axis': axis,
                'pixel_offset_px': tick,
                'real_distance_cm': dist_cm,
                'cal_height_cm': height_cm,
                'theta_rad': theta,
            })

        print()

    cv2.destroyAllWindows()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=['axis', 'pixel_offset_px', 'real_distance_cm',
                        'cal_height_cm', 'theta_rad'],
        )
        writer.writeheader()
        writer.writerows(records)

    x_count = sum(1 for r in records if r['axis'] == 'x')
    y_count = sum(1 for r in records if r['axis'] == 'y')
    print(f"Saved {x_count} +Y points and {y_count} +X points → {csv_path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Collect pixel-to-angle lens calibration data."
    )
    parser.add_argument('--source', default='0',
                        help='Camera index or stream URL (default: 0)')
    parser.add_argument('--height-cm', type=float, default=DEFAULT_HEIGHT_CM,
                        help=f'Camera height above ground in cm (default: {DEFAULT_HEIGHT_CM})')
    parser.add_argument('--step', type=int, default=DEFAULT_STEP,
                        help=f'Pixel interval between ticks (default: {DEFAULT_STEP})')
    parser.add_argument('--output', default=str(DEFAULT_CSV),
                        help=f'Output CSV (default: {DEFAULT_CSV})')
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    cap = open_capture(source)
    if not cap.isOpened():
        print(f"Cannot open source {source!r}")
        sys.exit(1)

    try:
        run(cap, args.height_cm, args.step, Path(args.output))
    finally:
        cap.release()


if __name__ == '__main__':
    main()
