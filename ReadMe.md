# Rocket/Drone Ground Station

## System Overview

Autonomous UAV ground station with computer vision targeting. The aircraft flies a default mission plan, the downward-facing camera detects an object of interest, and the ground station dynamically redirects the aircraft to investigate, then returns it to the original mission.

## Data Flow

```
                              ┌── QGroundControl (UDP 14550)  — monitoring UI
Flight Controller ──── router.py
   (ArduPilot)                 ├── Ground station   (UDP 14551)  — vision + telemetry
                               ├── Flight control   (UDP 14552)  — WASD + commands
                               └── Diagnostics      (UDP 14553)  — passive monitors
```

**Telemetry IN:** `router.py` owns the serial link and fans MAVLink out to QGroundControl, Python, and diagnostics listeners.

**Commands OUT:** Python sends directly to `router.py` → aircraft. QGroundControl is a peer, not an upstream hop.

## Flight Modes

| Mode | Trigger | Description |
|------|---------|-------------|
| AUTO | Default | Craft follows QGC mission plan autonomously |
| GUIDED | Vision detects target | Python sends GPS coordinates, craft redirects. Resumes AUTO when done. |
| MANUAL/QSTABILIZE | WASD override | Python sends RC overrides directly to `router.py` |

GUIDED mode and WASD overrides are sent **Python → router.py → craft**. They do not route through QGroundControl.

## Vision Pipeline

1. Capture video from the downward camera (`cv2.VideoCapture(0)`, `1920x1080 @ 10fps`)
2. Process frames to detect object of interest
3. Convert pixel offset → GPS coordinates using telemetry (altitude, heading, camera FOV)
4. Switch craft to GUIDED mode
5. Send `SET_POSITION_TARGET_GLOBAL_INT` with target GPS coords (~1Hz)
6. Switch back to AUTO when done

## Vision Module Layout

The current vision code is organized so `video_capture.py` stays as a thin live-view entrypoint and the actual pipeline can be reused by a future `rocket.py` or autonomy loop.

```text
vision/
  config.py       fixed sample telemetry scenarios + default camera model
  detector.py     image-space target detection, currently color-blob based
  models.py       shared dataclasses for detections, telemetry, and coordinates
  projection.py   pixel -> camera-relative ground coordinate math
  telemetry.py    provider interface + static sample provider
  tracker.py      black-box pipeline: frame in -> latest relative coordinate out
video_capture.py  live camera runner + debug overlay
```

Current black-box boundary:
- `VisionTracker.process_frame(frame)` returns the latest detection and camera-relative `xyz`
- `TelemetryProvider.get_latest()` is the swap point for future live MAVLink telemetry

That keeps the first bench test simple:
1. Detect the taped ground target in pixel space
2. Confirm the centroid overlay is correct
3. Feed fixed altitude/pitch/roll values
4. Read out a camera-relative coordinate for guidance logic

Current bench defaults:
- target marker color: `blue`
- target detector type: HSV color segmentation, not a trained ML model
- camera model: `CADDXFPV Walksnail Avatar Pro Kit`
- default FOV assumption: infer horizontal/vertical FOV from the published `160°` diagonal FOV on a `16:9` frame

## Setup

Find the USB port:
```bash
ls /dev/cu.*
```

Activate the virtual environment:
```bash
source venv/bin/activate
```

Start the integrated launcher before opening QGroundControl:
```bash
python3 startup.py
```

`startup.py` currently:
1. Auto-detects the serial port (default `/dev/cu.usbserial-0001`, 57600 baud)
2. Starts `router.py`
3. Waits for a heartbeat on UDP 14551
4. Starts the ground-station GUI
5. Starts the WASD flight-control loop in the main terminal

Startup order:
1. Power on the aircraft
2. Plug in the radio adapter
3. Close QGroundControl if it is open
4. Run `python3 startup.py`
5. Let the ground station auto-launch QGroundControl, or open it manually after startup is ready

If the serial device is not auto-detected, verify with `ls /dev/cu.*` and update `PREFERRED_SERIAL_PORT` in `startup.py`.

## QGroundControl Setup

Use `router.py` as the single MAVLink hub. Do not connect QGroundControl directly to the serial device.

Current local UDP outputs:
- `udp:127.0.0.1:14550` for QGroundControl
- `udp:127.0.0.1:14551` for the ground station / telemetry listeners
- `udp:127.0.0.1:14552` for flight control / command injection
- `udp:127.0.0.1:14553` for passive diagnostics monitors

If QGroundControl has connection issues:
- Disable serial auto-connect so QGroundControl does not try to open the device directly
- Connect via UDP on port `14550`
- Keep `startup.py` / `router.py` running the whole time; QGroundControl should attach to UDP, not replace the serial owner

## Scripts

| Script | Purpose |
|--------|---------|
| `startup.py` | Main launcher — starts `router.py`, the ground station, and manual flight control |
| `router.py` | Minimal serial-to-UDP MAVLink fanout and command return path |
| `main.py` | WASD manual flight control + HUD. `F` toggles wing deploy on `SERVO8` |
| `rocket_telemetry.py` | Passive telemetry monitor (altitude, GPS, orientation, battery) |
| `ground_station.py` | Vision UI + MAVLink telemetry display + QGroundControl launcher |
| `mavlink_interface/param_manager.py` | Push `config/params.yaml` to FC. Run instead of startup.py |
| `mavlink_interface/servo_output_monitor.py` | Watch `SERVO_OUTPUT_RAW` to verify what the FC thinks it is driving on outputs 1-8 |
| `mavlink_interface/cleanup_ports.py` | Kill stale MAVLink/router Python processes holding the serial device or UDP ports |
| `calibration/calibrate.py` | One-time FC orientation + gyro/baro calibration (run once) |
| `calibration/compass_diagnose.py` | Streams magnetometer data and plots sphere fit for diagnosis |

## Parameters

Non-default ArduPilot parameters live in `config/params.yaml`. Push them to the FC with:

```bash
python3 mavlink_interface/param_manager.py
```

Run this instead of `startup.py`, because it connects directly to `/dev/cu.usbserial-0001`.

Use `--verify-only` to read back and check without writing:

```bash
python3 mavlink_interface/param_manager.py --verify-only
```

Calibration parameters (INS, COMPASS offsets, AHRS orientation/trim) are never written by this script.

Current output mapping of note:
- `SERVO8` is the wing deployment output
- `SERVO8_FUNCTION: 0`
- `SERVO8_TRIM: 1000`
- `SERVO8_MIN/MAX: 1000/2000`

## Diagnostics

Free the serial device and local MAVLink UDP ports if a stale Python/router process is stuck:

```bash
python3 mavlink_interface/cleanup_ports.py
```

Watch flight-controller output commands directly from MAVLink:

Direct serial, when `startup.py` is not running:
```bash
python3 mavlink_interface/servo_output_monitor.py
```

Passive monitor through the running router:
```bash
python3 mavlink_interface/servo_output_monitor.py --connect udp:127.0.0.1:14553
```

This is useful for separating:
- FC output command issues
- downstream wiring / ESC board / servo path issues

If `SERVO_OUTPUT_RAW` shows a channel changing but the hardware does not move, the problem is downstream of the FC logic.

## Manual Control Notes

`main.py` now reduces idle MAVLink traffic so QGroundControl stays responsive on the lossy 57600-baud radio link:
- idle GCS heartbeats are low-rate
- RC overrides are only transmitted during active manual input
- release-to-center uses a short neutral burst before releasing override ownership

Wing deployment uses `MAV_CMD_DO_SET_SERVO` on `SERVO8` with the original simple retry pattern:
- 8 sends
- 50 ms gap
- `1000` = stowed
- `2000` = deployed

## Current Status

Video capture is working on camera index 0. The main system has been migrated off MAVProxy to `router.py`, and manual-control traffic was reduced so QGroundControl responsiveness is much better on the radio link. Hardware testing in this session isolated wing deployment to `SERVO8`; output 5 was confirmed to be a downstream hardware-path problem rather than a software/config issue.

## Build Order (remaining)

1. Telemetry pipeline — get altitude, GPS, heading reliably into Python
2. Mode controller — manage AUTO / GUIDED / MANUAL switching from one script
3. Vision → GPS math — pixel offset + altitude + FOV → target lat/lon
4. Close the loop — feed lat/lon into GUIDED mode commands
