"""
ground_station.py

Integrated ground station GUI.

  [Vision Feed   ]
  [Detection Mask]
  [Terminal Log — full width]

QGroundControl is launched, positioned to the right, and resized to fill
the remaining screen space so it sits flush against this window.

Usage (normally launched by startup.py):
    python3 ground_station.py [--source 0] [--no-qgc]
"""

import argparse
import math
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import tkinter as tk

try:
    from PIL import Image, ImageTk
except ImportError:
    sys.exit("Pillow is required:  pip install -r requirements.txt")

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vision import (
    DEFAULT_CAMERA_MODEL,
    GroundProjector,
    StaticTelemetryProvider,
    TELEMETRY_SCENARIOS,
    VisionTracker,
)
from vision.config import (
    DEFAULT_TARGET_COLOR,
    DEFAULT_TARGET_MIN_AREA_PX,
    TARGET_COLOR_RANGES,
)
from vision.detector import build_detector

# ── Layout ─────────────────────────────────────────────────────────────────
PADDING     = 6
TITLE_H     = 20        # height of each panel's title bar
QGC_APP     = "QGroundControl"

# ── Palette ────────────────────────────────────────────────────────────────
BG      = "#0d1117"
PANEL   = "#161b22"
BORDER  = "#30363d"
FG      = "#e6edf3"
DIM     = "#8b949e"
GREEN   = "#3fb950"
ORANGE  = "#d29922"
RED     = "#f85149"

# ── Clean exit when startup.py sends SIGTERM ────────────────────────────────
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))


# ───────────────────────────────────────────────────────────────────────────
# Helpers
# ───────────────────────────────────────────────────────────────────────────

def cv2_to_tk(frame: np.ndarray, w: int, h: int) -> ImageTk.PhotoImage:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(image=Image.fromarray(rgb).resize((w, h), Image.BILINEAR))


# ───────────────────────────────────────────────────────────────────────────
# Vision worker  (background thread)
# ───────────────────────────────────────────────────────────────────────────

class VisionWorker:
    def __init__(self, video_source, tracker, maxsize: int = 2):
        self.video_source = video_source
        self.tracker      = tracker
        self.frame_q      = queue.Queue(maxsize=maxsize)
        self.mask_q       = queue.Queue(maxsize=maxsize)
        self._stop        = threading.Event()
        self._thread      = threading.Thread(target=self._run, daemon=True)

    def start(self):  self._thread.start()
    def stop(self):   self._stop.set()

    def _run(self):
        import platform
        backend = (cv2.CAP_AVFOUNDATION
                   if platform.system() == "Darwin" and isinstance(self.video_source, int)
                   else cv2.CAP_ANY)
        cap = cv2.VideoCapture(self.video_source, backend)
        if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            self._push(self.frame_q, self._err_frame(f"Cannot open {self.video_source!r}"))
            return
        while not self._stop.is_set():
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.02)
                continue
            result = self.tracker.process_frame(frame)
            self._annotate(frame, result)
            self._push(self.frame_q, frame)
            mask = self.tracker.build_debug_mask(frame)
            if mask is not None:
                self._push(self.mask_q, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
        cap.release()

    @staticmethod
    def _push(q, item):
        try:
            q.put_nowait(item)
        except queue.Full:
            try: q.get_nowait()
            except queue.Empty: pass
            try: q.put_nowait(item)
            except queue.Full: pass

    @staticmethod
    def _annotate(frame, result):
        det = result.detection
        if det is None:
            cv2.putText(frame, "no target", (14, 34),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 80, 220), 2)
            return
        cv2.rectangle(frame, (det.x_px, det.y_px),
                      (det.x_px + det.width_px, det.y_px + det.height_px), (0, 255, 80), 2)
        cv2.circle(frame, (int(det.center_x_px), int(det.center_y_px)), 6, (0, 255, 255), -1)
        if result.coordinate is not None:
            c = result.coordinate
            cv2.putText(frame,
                        f"{det.label}  xyz=({c.x_m:.2f}, {c.y_m:.2f}, {c.z_m:.2f}) m",
                        (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 80), 2)

    @staticmethod
    def _err_frame(msg: str) -> np.ndarray:
        f = np.zeros((360, 640, 3), dtype=np.uint8)
        f[:] = (18, 18, 28)
        cv2.putText(f, msg, (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 80, 220), 2)
        return f


# ───────────────────────────────────────────────────────────────────────────
# Telemetry receiver  (background thread)
# ───────────────────────────────────────────────────────────────────────────

class TelemetryReceiver:
    """Passive MAVLink listener on UDP 14551."""

    def __init__(self, connection_string: str = "udp:127.0.0.1:14551"):
        self._conn_str = connection_string
        self._lock     = threading.Lock()
        self._data: dict = {
            "link_ok": False, "armed": False, "mode": "UNKNOWN",
            "alt": 0.0, "lat": 0.0, "lon": 0.0,
            "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
            "airspeed": 0.0, "climb": 0.0,
            "batt_v": 0.0, "batt_pct": -1,
        }
        self._prev_mode  = "UNKNOWN"
        self._prev_armed = False
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self): self._thread.start()

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def _run(self):
        try:
            conn = mavutil.mavlink_connection(self._conn_str)
            hb   = conn.wait_heartbeat(timeout=30)
            if hb is None:
                return
            with self._lock:
                self._data["link_ok"] = True
            print("[link] MAVLink connected")
            while not self._stop.is_set():
                msg = conn.recv_match(blocking=True, timeout=0.5)
                if msg is None or msg.get_srcSystem() != conn.target_system:
                    continue
                t = msg.get_type()
                with self._lock:
                    d = self._data
                    if t == "HEARTBEAT":
                        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        mode  = d["mode"]
                        for name, mid in conn.mode_mapping().items():
                            if mid == msg.custom_mode:
                                mode = name
                        if armed != self._prev_armed:
                            print(f"[arm]  {'ARMED' if armed else 'disarmed'}")
                            self._prev_armed = armed
                        if mode != self._prev_mode:
                            print(f"[mode] {mode}")
                            self._prev_mode = mode
                        d["armed"] = armed
                        d["mode"]  = mode
                    elif t == "ATTITUDE":
                        d["pitch"] = math.degrees(msg.pitch)
                        d["roll"]  = math.degrees(msg.roll)
                        d["yaw"]   = math.degrees(msg.yaw)
                    elif t == "GLOBAL_POSITION_INT":
                        d["alt"] = msg.relative_alt / 1000.0
                        d["lat"] = msg.lat / 1e7
                        d["lon"] = msg.lon / 1e7
                    elif t == "VFR_HUD":
                        d["airspeed"] = msg.airspeed
                        d["climb"]    = msg.climb
                    elif t == "SYS_STATUS":
                        d["batt_v"]   = msg.voltage_battery / 1000.0
                        d["batt_pct"] = msg.battery_remaining
        except Exception as exc:
            print(f"[link] error: {exc}")


# ───────────────────────────────────────────────────────────────────────────
# Main application
# ───────────────────────────────────────────────────────────────────────────

class GroundStationApp:
    _REFRESH_MS = 40     # ~25 fps

    def __init__(self, root: tk.Tk, vision_worker: VisionWorker,
                 telem: TelemetryReceiver, vis_w: int, vis_h: int):
        self.root   = root
        self.vision = vision_worker
        self.telem  = telem
        self.vis_w  = vis_w
        self.vis_h  = vis_h

        # Keep PhotoImage references so GC doesn't collect them
        self._img_main = None
        self._img_mask = None

        self._build_ui()
        self.root.after(self._REFRESH_MS, self._tick)

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        r = self.root
        r.configure(bg=BG)
        r.title("Rocket Ground Station")
        r.resizable(False, False)

        # Top area: vision panels
        top = tk.Frame(r, bg=BG)
        top.grid(row=0, column=0, padx=PADDING, pady=(PADDING, 0), sticky="nw")

        blank = ImageTk.PhotoImage(Image.new("RGB", (self.vis_w, self.vis_h), (0, 0, 0)))
        self._img_main = blank
        self._img_mask = blank

        self._main_lbl = self._titled_image(top, "VISION FEED",     blank, pady_bot=PADDING // 2)
        self._mask_lbl = self._titled_image(top, "DETECTION MASK",  blank, pady_bot=0)

    @staticmethod
    def _titled_image(parent, title: str, blank_img, *, pady_bot: int) -> tk.Label:
        outer = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        outer.pack(pady=(0, pady_bot))
        tk.Label(outer, text=title, bg=PANEL, fg=DIM,
                 font=("Courier", 9, "bold"), anchor="w").pack(fill="x", padx=6, pady=(3, 0))
        lbl = tk.Label(outer, image=blank_img, bg="#000000")
        lbl.pack()
        return lbl

    # ── Periodic tick ──────────────────────────────────────────────────────

    def _tick(self):
        self._update_vision()
        self.root.after(self._REFRESH_MS, self._tick)

    def _update_vision(self):
        try:
            frame = self.vision.frame_q.get_nowait()
            img   = cv2_to_tk(frame, self.vis_w, self.vis_h)
            self._img_main = img
            self._main_lbl.configure(image=img)
        except queue.Empty:
            pass
        try:
            mask = self.vision.mask_q.get_nowait()
            img  = cv2_to_tk(mask, self.vis_w, self.vis_h)
            self._img_mask = img
            self._mask_lbl.configure(image=img)
        except queue.Empty:
            pass

# ───────────────────────────────────────────────────────────────────────────
# QGroundControl launcher + positioning
# ───────────────────────────────────────────────────────────────────────────

def _launch_qgc(gs_width: int, screen_w: int, screen_h: int) -> None:
    """Open QGroundControl and resize it to fill the remaining screen space."""
    try:
        subprocess.Popen(["open", "-a", QGC_APP])
    except Exception as e:
        print(f"[qgc] Could not launch {QGC_APP}: {e}")
        return

    qgc_x = gs_width + 4
    qgc_w = screen_w - qgc_x

    time.sleep(4)    # wait for QGC to finish opening

    # Position and resize QGC to fill the right portion of the screen.
    script = (
        f'tell application "{QGC_APP}" to activate\n'
        f'delay 0.5\n'
        f'tell application "System Events"\n'
        f'  tell process "{QGC_APP}"\n'
        f'    set position of window 1 to {{{qgc_x}, 0}}\n'
        f'    set size of window 1 to {{{qgc_w}, {screen_h}}}\n'
        f'  end tell\n'
        f'end tell\n'
    )
    try:
        subprocess.Popen(["osascript", "-e", script])
        print(f"[qgc] positioned at x={qgc_x}, size {qgc_w}×{screen_h}")
    except Exception as e:
        print(f"[qgc] Could not position {QGC_APP}: {e}")


# ───────────────────────────────────────────────────────────────────────────
# Entry point
# ───────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Rocket ground station GUI")
    p.add_argument("--source", default="0", help="Camera index or stream URL")
    p.add_argument("--target-color", default=DEFAULT_TARGET_COLOR,
                   choices=sorted(TARGET_COLOR_RANGES))
    p.add_argument("--min-area", type=int, default=DEFAULT_TARGET_MIN_AREA_PX)
    p.add_argument("--scenario", default="bench_nadir",
                   choices=sorted(TELEMETRY_SCENARIOS))
    p.add_argument("--no-qgc", action="store_true",
                   help="Skip launching QGroundControl")
    return p.parse_args()


def normalize_source(s: str):
    try: return int(s)
    except ValueError: return s


def main():
    args = parse_args()

    # Measure screen
    root = tk.Tk()
    root.withdraw()
    screen_w = root.winfo_screenwidth()
    screen_h = root.winfo_screenheight()

    # Vision panels: leave ≥600 px for QGC on the right.
    vis_w = min(560, screen_w - 600 - PADDING * 2)
    vis_w = max(vis_w, 300)
    vis_h = vis_w * 9 // 16

    # Position at top-left; let tkinter size the window to fit the panels.
    root.geometry("+0+0")
    root.deiconify()

    # Vision pipeline
    detector  = build_detector(args.target_color, min_area_px=args.min_area)
    telem_prov = StaticTelemetryProvider.from_scenario(args.scenario)
    projector  = GroundProjector(DEFAULT_CAMERA_MODEL)
    tracker    = VisionTracker(detector, telem_prov, projector)
    vision     = VisionWorker(normalize_source(args.source), tracker)

    # Telemetry receiver (logs arm/mode events to the VSCode terminal)
    telem = TelemetryReceiver()

    # Build GUI
    GroundStationApp(root, vision, telem, vis_w, vis_h)

    vision.start()
    telem.start()

    # Measure actual window width after layout settles, then position QGC.
    root.update_idletasks()
    win_w = root.winfo_width()

    if not args.no_qgc:
        threading.Thread(
            target=_launch_qgc, args=(win_w, screen_w, screen_h), daemon=True
        ).start()

    print(f"[gs] vis {vis_w}×{vis_h}  |  QGC → x={win_w + 4}, width={screen_w - win_w - 4}")

    try:
        root.mainloop()
    finally:
        vision.stop()


if __name__ == "__main__":
    main()
