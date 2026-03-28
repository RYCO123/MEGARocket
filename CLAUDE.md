# Rocket/Drone Ground Station — CLAUDE.md

## Project Overview

This is the **ground station side** of an autonomous UAV (fixed-wing drone/rocket) system. The aircraft runs ArduPilot firmware and is controlled via MAVLink protocol. The big-picture mission:

1. Aircraft flies a default mission plan set in QGroundControl
2. A downward-facing camera streams video to the ground station
3. Ground station processes video frames in Python to detect an **object of interest**
4. When found, the ground station sends a **mission plan update** to the aircraft via MAVLink
5. Aircraft modifies its course to investigate/converge on the target

**Current status (as of Mar 2026):** Video is visible in QuickTime Player (circuits built, hardware connected). Next step is getting video into Python for processing.

---

## Hardware Stack

- **Aircraft:** Fixed-wing UAV running ArduPilot (VTOL capable — has QSTABILIZE mode)
- **Flight controller:** Connected via USB serial (`/dev/cu.usbmodem*`) at 115200 baud
- **Ground station:** Mac (ryancody's MacBook Air)
- **Camera:** Downward-facing, currently outputting video visible in QuickTime Player
- **Communication:** MAVLink over serial → MAVProxy UDP bridge

## Software Stack

| Layer | Tool | Role |
|-------|------|------|
| Flight firmware | ArduPilot | Runs on aircraft |
| Ground middleware | MAVProxy | USB serial → UDP bridge + multiplexer |
| Ground control UI | QGroundControl | Mission planning, monitoring (UDP 14550) |
| Ground station code | Python + pymavlink | Vision + autonomy (UDP 14551) |

## Data Flow — Critical Architecture

```
                         ┌── QGroundControl (UDP 14550)  — monitoring only
Flight Controller ──── MAVProxy
   (ArduPilot)           └── Python scripts  (UDP 14551)  — vision + commands
```

- **Telemetry IN:** MAVProxy fans out to BOTH QGC and Python simultaneously
- **Commands OUT:** Python → MAVProxy → craft (direct, does NOT go through QGC)
- QGC and Python are peers. Neither is upstream of the other.
- GUIDED mode commands and WASD RC overrides go Python → MAVProxy → craft

## Flight Modes

- **AUTO** — craft follows QGC mission plan
- **GUIDED** — Python sends `SET_POSITION_TARGET_GLOBAL_INT` (~1Hz) with target GPS; craft redirects. Must resend every second or craft stops. Switch back to AUTO when done.
- **MANUAL/QSTABILIZE** — Python sends RC channel overrides

---

## Key Files

| File | Purpose |
|------|---------|
| `main.py` | Mission control — keyboard flight control, RC overrides, HUD display |
| `rocket_telemtry.py` | Telemetry receiver — reads serial, prints altitude/GPS/orientation/battery |
| `mav_proxy.txt` | MAVProxy startup commands |
| `requirements.txt` | Python deps: pymavlink, pynput, pyserial, MAVProxy |
| `mav.parm` | ArduPilot parameter file (cruise 12 m/s, max 22 m/s, 3300mAh battery) |
| `mav.tlog` | Flight telemetry log (binary) |

---

## MAVProxy Setup

Find the USB port:
```bash
ls /dev/cu.usbmodem*
```

Start MAVProxy (bridges serial to two UDP ports):
```bash
python3 -m MAVProxy.mavproxy --master=/dev/cu.usbserial-0001 --baudrate 57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```
Note: if port changes, find it with `ls /dev/cu.*` — look for `cu.usbserial-*`

- Port **14550** → QGroundControl
- Port **14551** → Python scripts (main.py)

**Startup order:**
1. Close QGroundControl
2. Run MAVProxy command above
3. Open QGroundControl
4. Run Python script in separate terminal

---

## MAVLink Connection (Python)

```python
from pymavlink import mavutil
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
```

Arm bypass code: `21196`

Flight modes used: `MANUAL`, `QSTABILIZE`, `FBWA`

RC channels: Roll=Ch1, Pitch=Ch2, Yaw=Ch4

---

## Image Processing Pipeline (in progress)

**Goal:** Capture video feed in Python → detect object of interest → update MAVLink mission

**Status:** Video is available in QuickTime Player. Need to capture it via Python (likely via `AVFoundation`/`cv2.VideoCapture` on macOS).

**Camera device indices (confirmed):**
- Index 0 — drone/rocket downward camera (1920x1080 @ 10fps) ← USE THIS
- Index 1 — MacBook built-in camera
- Index 2 — iPhone continuity camera

**Next steps after capture working:**
1. Frame-by-frame processing with OpenCV
2. Object detection (TBD — color masking, YOLO, or similar)
3. Convert pixel location → GPS coordinates using altitude + camera FOV
4. Send `SET_MISSION_ITEM` or `MISSION_ITEM_INT` MAVLink messages to update waypoint

---

## Virtual Environment

```bash
source venv/bin/activate
```

---

## Notes

- `rocket_telemtry.py` has a typo in the filename ("telemtry")
- The `.gitignore` only excludes `venv/`
- `mav.tlog` / `mav.tlog.raw` are binary flight logs, not text
