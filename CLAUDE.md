# Rocket/Drone Ground Station — CLAUDE.md

## Project Overview

This is the **ground station side** of an autonomous UAV (fixed-wing drone/rocket) system. The aircraft runs ArduPilot firmware and is controlled via MAVLink protocol. The big-picture mission:

1. Aircraft flies a default mission plan set in QGroundControl
2. A downward-facing camera streams video to the ground station
3. Ground station processes video frames in Python to detect an **object of interest**
4. When found, the ground station sends a **mission plan update** to the aircraft via MAVLink
5. Aircraft modifies its course to investigate/converge on the target

**Current status (as of Apr 2026):** Video is working again on camera index 0 after a full Walksnail reboot. `startup.py` now launches MAVProxy, applies a lean `SR1_*` and `SR2_*` telemetry profile, then starts `rocket_telemtry.py` and `video_capture.py`. The main unresolved issue is the MAVLink link quality over the radio path: QGroundControl reported about 96% loss rate during testing, and parameter download over that link was extremely slow.

---

## Hardware Stack

- **Aircraft:** Fixed-wing UAV running ArduPilot (VTOL capable — has QSTABILIZE mode)
- **Flight controller / ground link:** Ranger Micro USB serial at `/dev/cu.usbserial-0001`, currently run through MAVProxy at 460800 baud
- **Ground station:** Mac (ryancody's MacBook Air)
- **Camera:** Downward-facing, captured in Python via `cv2.VideoCapture(0)`
- **Communication:** MAVLink over ELRS via Ranger Micro → MAVProxy UDP bridge

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
| `rocket_telemtry.py` | Passive telemetry HUD — listens on UDP 14551 and displays altitude/GPS/orientation/battery |
| `mav_proxy.txt` | MAVProxy startup commands |
| `start_mavproxy.sh` | Current working MAVProxy launcher |
| `startup.py` | Main launcher — starts MAVProxy, fetches params, applies `SR1_*`/`SR2_*`, launches telemetry + video |
| `requirements.txt` | Python deps: pymavlink, pynput, pyserial, MAVProxy |
| `mav.parm` | ArduPilot parameter file (cruise 12 m/s, max 22 m/s, 3300mAh battery) |
| `mav.tlog` | Flight telemetry log (binary) |

---

## MAVProxy Setup

Find the serial port:
```bash
ls /dev/cu.*
```

Start MAVProxy (bridges serial to two UDP ports):
```bash
./start_mavproxy.sh
```

That script currently runs:
```bash
python3 -m MAVProxy.mavproxy --master=/dev/cu.usbserial-0001 --baudrate 460800 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

- Port **14550** → QGroundControl
- Port **14551** → Python scripts (main.py)

**Startup order:**
1. Power on the aircraft
2. Plug the Ranger Micro into the Mac
3. Close QGroundControl
4. Run `./start_mavproxy.sh`
5. Open QGroundControl
6. Run a Python script in a separate terminal

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

**Status:** Video capture in Python is working again on index 0. A temporary black-screen issue was traced to the Walksnail chain rather than Python/OpenCV, because QuickTime also showed black until both Walksnail ends were rebooted.

**Camera device indices (confirmed):**
- Index 0 — drone/rocket downward camera (1920x1080 @ 10fps) ← USE THIS
- Index 1 — MacBook built-in camera
- Index 2 — iPhone continuity camera

**Video diagnosis notes (Apr 2026):**
- `video_capture.py` is hardcoded back to `cv2.VideoCapture(0)`.
- macOS camera permissions were verified for VS Code/Terminal during debugging.
- QuickTime saw the capture device but also showed black, which ruled out Python/OpenCV as the root cause.
- Rebooting the Walksnail system restored video.

## Telemetry Diagnosis (Apr 2026)

- `startup.py` now starts MAVProxy with `--streamrate=-1` so MAVProxy does not impose a broad default stream profile.
- `startup.py` fetches parameters with `param_fetch_all()` instead of requesting `MAV_DATA_STREAM_ALL`.
- After parameter fetch, `startup.py` writes a low-bandwidth profile to both `SR1_*` and `SR2_*`:
  - `POSITION = 2`
  - `EXT_STAT = 1`
  - `EXTRA1 = 5`
  - `EXTRA2 = 2`
  - `RAW_SENS = 0`
  - `RC_CHAN = 0`
  - `RAW_CTRL = 0`
- `rocket_telemtry.py` is now passive. It does not send `REQUEST_DATA_STREAM` or `MAV_CMD_SET_MESSAGE_INTERVAL`; it only listens and reports what arrives.
- Live test result: QGroundControl `Loss rate` was about `96%` over the radio link.
- Live test result: parameter download over the radio path was only about `1-3 params/sec` after the initial burst.
- Next diagnostic step: connect the flight controller directly to the Mac over USB and compare QGC loss rate and parameter download speed. If USB is healthy, the bottleneck is the ELRS/radio telemetry path rather than Python or MAVProxy.

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
