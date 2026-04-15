# Rocket/Drone Ground Station — CLAUDE.md

## Project Overview

This is the **ground station side** of an autonomous UAV (fixed-wing drone/rocket) system. The aircraft runs ArduPilot firmware and is controlled via MAVLink protocol. The big-picture mission:

1. Aircraft flies a default mission plan set in QGroundControl
2. A downward-facing camera streams video to the ground station
3. Ground station processes video frames in Python to detect an **object of interest**
4. When found, the ground station sends a **mission plan update** to the aircraft via MAVLink
5. Aircraft modifies its course to investigate/converge on the target

**Current status (as of Apr 2026):** Switched from MAVProxy to mavlink-router as the serial → UDP bridge. mavlink-router is C++ with near-zero forwarding overhead; MAVProxy (Python) was confirmed as a bottleneck — QGC direct on serial was significantly faster than through MAVProxy. Radio link has ~96% packet loss at 57600 baud; this is a hardware constraint.

---

## Hardware Stack

- **Aircraft:** Fixed-wing UAV running ArduPilot (VTOL capable — has QSTABILIZE mode)
- **Flight controller / ground link:** USB serial at `/dev/cu.usbserial-0001`, 57600 baud
- **Ground station:** Mac (ryancody's MacBook Air)
- **Camera:** Downward-facing, captured in Python via `cv2.VideoCapture(0)`
- **Communication:** MAVLink over analog radio antennas → mavlink-router UDP bridge

## Software Stack

| Layer | Tool | Role |
|-------|------|------|
| Flight firmware | ArduPilot | Runs on aircraft |
| Ground middleware | router.py | USB serial → UDP bridge + multiplexer (minimal Python, no MAVProxy overhead) |
| Ground control UI | QGroundControl | Mission planning, monitoring (UDP 14550) |
| Ground station code | Python + pymavlink | Vision + autonomy (UDP 14551/14552) |

## Data Flow — Critical Architecture

```
                              ┌── QGroundControl (UDP 14550)  — monitoring only
Flight Controller ── router.py
   (ArduPilot)                ├── Python telemetry  (UDP 14551)  — vision + autonomy
                              └── Python control    (UDP 14552)  — RC overrides + commands
```

- **Telemetry IN:** mavlink-router fans out to QGC and both Python ports simultaneously
- **Commands OUT:** Python → mavlink-router → craft (direct, does NOT go through QGC)
- QGC and Python are peers. Neither is upstream of the other.
- GUIDED mode commands and WASD RC overrides go Python → mavlink-router → craft

## Flight Modes

- **AUTO** — craft follows QGC mission plan
- **GUIDED** — Python sends `SET_POSITION_TARGET_GLOBAL_INT` (~1Hz) with target GPS; craft redirects. Must resend every second or craft stops. Switch back to AUTO when done.
- **MANUAL/QSTABILIZE** — Python sends RC channel overrides

---

## Key Files

| File | Purpose |
|------|---------|
| `main.py` | Mission control — keyboard flight control, RC overrides, HUD display |
| `ground_station.py` | GUI — vision feed + detection mask panels, auto-launches QGC |
| `startup.py` | Main launcher — starts mavlink-router, ground station, RocketCommander |
| `rocket_telemetry.py` | Passive telemetry HUD — listens on UDP 14551 |
| `mavlink_interface/param_manager.py` | Push `config/params.yaml` to FC (run instead of startup.py) |
| `config/params.yaml` | Non-default ArduPilot parameters |

---

## Router Setup

`router.py` is a minimal Python serial-to-UDP forwarder. No install needed — just pyserial (already in requirements.txt).

Find the serial port:
```bash
ls /dev/cu.*
```

Start everything (auto-detects serial port, 57600 baud):
```bash
python3 startup.py
```

- Port **14550** → QGroundControl
- Port **14551** → Python telemetry / ground station
- Port **14552** → RocketCommander (RC overrides + commands)

Router binds local ports 14540/14541/14542 and forwards to 14550/14551/14552.
Endpoint apps reply to the router's local port, completing the bidirectional link.

**Startup order:**
1. Power on the aircraft
2. Plug in the radio adapter
3. Close QGroundControl
4. Run `python3 startup.py`
5. Open QGroundControl (auto-launched by ground_station.py)

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

**Camera device indices (confirmed):**
- Index 0 — drone/rocket downward camera (1920x1080 @ 10fps) ← USE THIS
- Index 1 — MacBook built-in camera
- Index 2 — iPhone continuity camera

**Vision module layout:**
```
vision/
  config.py       fixed sample telemetry scenarios + default camera model
  detector.py     image-space target detection (HSV color segmentation)
  models.py       shared dataclasses for detections, telemetry, coordinates
  projection.py   pixel -> camera-relative ground coordinate math
  telemetry.py    provider interface + static sample provider
  tracker.py      pipeline: frame in -> latest relative coordinate out
```

**Next steps:**
1. Wire live MAVLink telemetry into `TelemetryProvider` (replace static provider)
2. Convert pixel offset → GPS coordinates using altitude + camera FOV
3. Send `SET_POSITION_TARGET_GLOBAL_INT` in GUIDED mode to redirect craft

---

## Radio / Telemetry Notes (Apr 2026)

- QGC direct on serial (57600 baud) is significantly faster than through MAVProxy — confirms MAVProxy Python overhead was the bottleneck, not the radio hardware itself
- Switched to router.py (minimal Python forwarder) which eliminates MAVProxy plugin/logging overhead
- Live radio link still has ~96% packet loss — hardware constraint, 57600 baud is non-negotiable
- QGC param download warning ("unable to retrieve full param list") is cosmetic — HUD, attitude, GPS, mode all work from streaming messages without the full param list
- Wing deploy (`F` key) sends `MAV_CMD_DO_SET_SERVO` 8× with 50ms gaps to survive packet loss

---

## Virtual Environment

```bash
source venv/bin/activate
```

---

## Notes

- `.gitignore` covers `venv/`, `__pycache__/`, `*.pyc`, `*.tlog`, `*.parm`, `camera_index.txt`, vision outputs
- `camera_index.txt` is auto-generated by startup.py to persist your video source selection between runs
