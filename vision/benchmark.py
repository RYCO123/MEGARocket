"""
vision/benchmark.py

10-point accuracy benchmark: compares the vision pipeline's predicted
x, y position against manually measured ground-truth values.

Setup:
  - Camera at 80 cm pointing straight down (same height as calibration)
  - Calibration CSV must exist (run calibrate_lens.py first)
  - Red marker on the ground

For each of 10 samples:
  1. Place the red marker at a known location
  2. Video shows the detection live — confirm bounding box is on the marker
  3. Measure the marker's actual position with a ruler (cm from center)
  4. Type:  x y   e.g.  5.2 12.1   (+x = right/stbd, +y = forward/nose)
  5. Press Enter to record

Results saved to vision/results/benchmark_YYYYMMDD_HHMMSS.csv

Usage:
    python3 vision/benchmark.py
    python3 vision/benchmark.py --source 0 --height-cm 80
"""

import argparse
import csv
import math
import platform
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vision import (
    CALIBRATION_CSV,
    CalibratedProjector,
    NullProjector,
    StaticTelemetryProvider,
    VisionTracker,
)
from vision.config import DEFAULT_TARGET_COLOR, DEFAULT_TARGET_MIN_AREA_PX
from vision.detector import build_detector

RESULTS_DIR       = Path(__file__).resolve().parent / "results"
DEFAULT_HEIGHT_CM = 80.0
DEFAULT_PITCH_DEG = -5.0
DEFAULT_ROLL_DEG  =  5.0
NUM_SAMPLES       = 10

# Colors (BGR)
C_CENTER  = (0,   0, 220)    # red
C_BOX     = (0, 255,  80)    # green
C_DOT     = (0, 255, 255)    # yellow
C_TEXT    = (0, 255,  80)
C_PENDING = (80, 80, 220)
C_DIM     = (120, 120, 120)


# ── Drawing ───────────────────────────────────────────────────────────────────

def draw_overlay(rot_frame, result, orig_w, sample_num, num_samples):
    out = rot_frame.copy()
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2

    # Red center +
    r = 12
    cv2.line(out, (cx - r, cy), (cx + r, cy), C_CENTER, 2)
    cv2.line(out, (cx, cy - r), (cx, cy + r), C_CENTER, 2)

    det = result.detection if result else None

    if det is not None:
        # Transform to rotated frame: 90° CCW → (px, py) → (py, orig_w-1-px)
        rcx = int(det.center_y_px)
        rcy = orig_w - 1 - int(det.center_x_px)
        rx  = det.y_px
        ry  = orig_w - 1 - det.x_px - det.width_px
        rw  = det.height_px
        rh  = det.width_px
        cv2.rectangle(out, (rx, ry), (rx + rw, ry + rh), C_BOX, 2)
        cv2.circle(out, (rcx, rcy), 6, C_DOT, -1)

        if result.coordinate is not None:
            c = result.coordinate
            cv2.putText(out,
                        f"pred  right={c.x_m:+.3f}m  fwd={c.y_m:+.3f}m",
                        (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_TEXT, 2)
        else:
            cv2.putText(out, "detection OK — no projection (check calibration)",
                        (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_PENDING, 1)
    else:
        cv2.putText(out, "no detection", (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, C_PENDING, 2)

    # Status bar
    cv2.putText(out,
                f"Sample {sample_num}/{num_samples}  |  type actual x y (cm) in terminal",
                (12, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_DIM, 1)
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
    frame = None
    for _ in range(4):
        ret, f = cap.read()
        if ret:
            frame = f
    return frame


def _stdin_reader(q):
    while True:
        line = sys.stdin.readline()
        if not line:
            break
        q.put(line.strip())


# ── Main benchmark loop ───────────────────────────────────────────────────────

def run_benchmark(cap, tracker, height_cm, pitch_deg, roll_deg, num_samples, csv_path):
    input_q: queue.Queue = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(input_q,), daemon=True).start()

    cv2.namedWindow("Benchmark", cv2.WINDOW_NORMAL)

    records = []
    sample_num = 1
    result = None
    orig_w = 1920   # updated from first frame
    frame  = None

    print(f"\n{'─'*55}")
    print(f" Benchmark: {num_samples}-point accuracy test")
    print(f" Height: {height_cm} cm   |   pitch: {pitch_deg:+.1f}°   roll: {roll_deg:+.1f}°")
    print(f" +x = right/stbd   +y = fwd/nose")
    print(f"{'─'*55}")
    _print_prompt(sample_num, num_samples)

    while sample_num <= num_samples:
        fresh = read_frame(cap)
        if fresh is not None:
            frame = fresh
            orig_w = frame.shape[1]
            result = tracker.process_frame(frame)
            rot = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow("Benchmark", draw_overlay(rot, result, orig_w, sample_num, num_samples))

        if cv2.waitKey(30) & 0xFF == ord('q'):
            print("\nAborted.")
            break

        # Check for typed input
        try:
            raw = input_q.get_nowait()
        except queue.Empty:
            continue

        if not raw:
            continue

        # Parse "x y" or "x,y"
        parts = raw.replace(',', ' ').split()
        if len(parts) != 2:
            print("    Enter two numbers: actual_x actual_y (cm)  e.g.  5.2 12.1")
            _print_prompt(sample_num, num_samples)
            continue
        try:
            actual_x_cm = float(parts[0])
            actual_y_cm = float(parts[1])
        except ValueError:
            print("    Invalid — enter two numbers in cm.")
            _print_prompt(sample_num, num_samples)
            continue

        # Must have a valid prediction to record
        if result is None or result.detection is None:
            print("    No marker detected — check camera, reposition, then re-enter.")
            _print_prompt(sample_num, num_samples)
            continue
        if result.coordinate is None:
            print("    Detection found but projection failed (calibration issue?).")
            _print_prompt(sample_num, num_samples)
            continue

        coord = result.coordinate
        det   = result.detection
        actual_x_m = actual_x_cm / 100.0
        actual_y_m = actual_y_cm / 100.0
        err_x_m    = coord.x_m - actual_x_m
        err_y_m    = coord.y_m - actual_y_m
        err_dist_m = math.hypot(err_x_m, err_y_m)
        actual_dist_m = math.hypot(actual_x_m, actual_y_m)
        err_pct = (err_dist_m / actual_dist_m * 100.0) if actual_dist_m > 1e-4 else 0.0

        records.append({
            'sample':       sample_num,
            'pixel_u':      f"{det.center_x_px:.1f}",
            'pixel_v':      f"{det.center_y_px:.1f}",
            'pitch_deg':    f"{pitch_deg:.2f}",
            'roll_deg':     f"{roll_deg:.2f}",
            'pred_x_m':     f"{coord.x_m:.4f}",
            'pred_y_m':     f"{coord.y_m:.4f}",
            'actual_x_cm':  f"{actual_x_cm:.2f}",
            'actual_y_cm':  f"{actual_y_cm:.2f}",
            'actual_x_m':   f"{actual_x_m:.4f}",
            'actual_y_m':   f"{actual_y_m:.4f}",
            'err_x_m':      f"{err_x_m:+.4f}",
            'err_y_m':      f"{err_y_m:+.4f}",
            'err_dist_m':   f"{err_dist_m:.4f}",
            'err_pct':      f"{err_pct:+.1f}",
        })

        print(f"  ✓  pred=({coord.x_m:+.3f}, {coord.y_m:+.3f})m  "
              f"actual=({actual_x_m:+.3f}, {actual_y_m:+.3f})m  "
              f"err={err_dist_m:.3f}m ({err_pct:+.1f}%)")

        sample_num += 1
        if sample_num <= num_samples:
            print()
            _print_prompt(sample_num, num_samples)

    cv2.destroyAllWindows()

    if not records:
        print("\nNo samples recorded.")
        return

    _save_and_summarise(records, csv_path)


def _print_prompt(sample_num, num_samples):
    print(f"\nSample {sample_num}/{num_samples}")
    print("  Place marker, measure its position, then type:  x y  (cm)  > ", end='', flush=True)


def _save_and_summarise(records, csv_path):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        'sample', 'pixel_u', 'pixel_v',
        'pitch_deg', 'roll_deg',
        'pred_x_m', 'pred_y_m',
        'actual_x_cm', 'actual_y_cm', 'actual_x_m', 'actual_y_m',
        'err_x_m', 'err_y_m', 'err_dist_m', 'err_pct',
    ]
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    errs = [float(r['err_dist_m']) for r in records]
    rmse = math.sqrt(sum(e**2 for e in errs) / len(errs))
    mean = sum(errs) / len(errs)
    max_e = max(errs)

    print(f"\n{'─'*55}")
    print(f" Results ({len(records)} samples)")
    print(f"   Mean error : {mean*100:.1f} cm")
    print(f"   RMSE       : {rmse*100:.1f} cm")
    print(f"   Max error  : {max_e*100:.1f} cm")
    print(f"   Saved      : {csv_path}")
    print(f"{'─'*55}\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Vision pipeline accuracy benchmark.")
    parser.add_argument('--source', default='0', help='Camera index or URL')
    parser.add_argument('--height-cm', type=float, default=DEFAULT_HEIGHT_CM,
                        help=f'Camera height in cm (default: {DEFAULT_HEIGHT_CM})')
    parser.add_argument('--pitch-deg', type=float, default=DEFAULT_PITCH_DEG,
                        help=f'Camera pitch in degrees (default: {DEFAULT_PITCH_DEG})')
    parser.add_argument('--roll-deg', type=float, default=DEFAULT_ROLL_DEG,
                        help=f'Camera roll in degrees (default: {DEFAULT_ROLL_DEG})')
    parser.add_argument('--target-color', default=DEFAULT_TARGET_COLOR,
                        help=f'Marker color (default: {DEFAULT_TARGET_COLOR})')
    parser.add_argument('--samples', type=int, default=NUM_SAMPLES,
                        help=f'Number of samples to collect (default: {NUM_SAMPLES})')
    parser.add_argument('--calibration-csv', default=str(CALIBRATION_CSV),
                        help='Path to lens calibration CSV')
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    # Build tracker (static telemetry — bench test at fixed height, no MAVLink)
    detector = build_detector(args.target_color, min_area_px=DEFAULT_TARGET_MIN_AREA_PX)
    telem    = StaticTelemetryProvider.from_values(
                    altitude_m=args.height_cm / 100.0,
                    pitch_deg=args.pitch_deg,
                    roll_deg=args.roll_deg)
    try:
        projector = CalibratedProjector(Path(args.calibration_csv))
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)
    tracker = VisionTracker(detector, telem, projector)

    cap = open_capture(source)
    if not cap.isOpened():
        print(f"Cannot open source {source!r}")
        sys.exit(1)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    # Encode attitude in filename so flat vs tilted runs are clearly distinct
    attitude  = f"p{args.pitch_deg:+.1f}_r{args.roll_deg:+.1f}".replace('+', 'p').replace('-', 'n').replace('.', 'd')
    csv_path  = RESULTS_DIR / f"benchmark_{attitude}_{timestamp}.csv"

    try:
        run_benchmark(cap, tracker, args.height_cm, args.pitch_deg, args.roll_deg,
                      args.samples, csv_path)
    finally:
        cap.release()


if __name__ == '__main__':
    main()
