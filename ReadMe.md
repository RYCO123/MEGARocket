# Rocket/Drone Ground Station

## System Overview

Autonomous UAV ground station with computer vision targeting. The aircraft flies a default mission plan, the downward-facing camera detects an object of interest, and the ground station dynamically redirects the aircraft to investigate, then returns it to the original mission.

## Data Flow

```
                         ┌── QGroundControl (UDP 14550)  — monitoring UI
Flight Controller ──── MAVProxy
   (ArduPilot)           └── Python scripts  (UDP 14551)  — vision + autonomy
```

**Telemetry IN:** MAVProxy receives telemetry from the aircraft and fans it out to both QGroundControl and Python.

**Commands OUT:** Python sends directly to MAVProxy → aircraft. QGroundControl does the same. Neither routes through the other; they are peers.

## Flight Modes

| Mode | Trigger | Description |
|------|---------|-------------|
| AUTO | Default | Craft follows QGC mission plan autonomously |
| GUIDED | Vision detects target | Python sends GPS coordinates, craft redirects. Resumes AUTO when done. |
| MANUAL/QSTABILIZE | WASD override | Python sends RC overrides directly to MAVProxy |

GUIDED mode and WASD overrides are sent **Python → MAVProxy → craft**. They do not route through QGroundControl.

## Vision Pipeline

1. Capture video from the downward camera (`cv2.VideoCapture(0)`, `1920x1080 @ 10fps`)
2. Process frames to detect object of interest
3. Convert pixel offset → GPS coordinates using telemetry (altitude, heading, camera FOV)
4. Switch craft to GUIDED mode
5. Send `SET_POSITION_TARGET_GLOBAL_INT` with target GPS coords (~1Hz)
6. Switch back to AUTO when done

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
1. Starts MAVProxy with `--streamrate=-1`
2. Fetches parameters with `param_fetch_all()`
3. Applies a lean telemetry profile to both `SR1_*` and `SR2_*`
4. Starts `rocket_telemtry.py`
5. Starts `video_capture.py`

The older `start_mavproxy.sh` script runs:
```bash
python3 -m MAVProxy.mavproxy --master=/dev/cu.usbserial-0001 --baudrate 460800 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Startup order:
1. Power on the aircraft
2. Plug the Ranger Micro into the Mac
3. Close QGroundControl if it is open
4. Run `python3 startup.py`
5. Open QGroundControl
6. `startup.py` launches telemetry and video automatically

If the serial device is not `/dev/cu.usbserial-0001`, update `start_mavproxy.sh` or run the MAVProxy command manually with the correct port.

## QGroundControl Wireless Setup

Use MAVProxy as the single MAVLink hub. Do not connect QGroundControl directly to the Ranger Micro serial device.

Data flow:
`Rocket <-> MAVProxy <-> QGroundControl`

`Rocket <-> MAVProxy <-> Python`

Current local UDP outputs:
- `udp:127.0.0.1:14550` for QGroundControl
- `udp:127.0.0.1:14551` for Python scripts

Recommended bring-up order:
1. Power on the aircraft
2. Plug in the Ranger Micro
3. Start `python3 startup.py`
4. Confirm telemetry is flowing in Python
5. Open QGroundControl

If QGroundControl has connection issues:
- Disable any serial auto-connect option so QGroundControl does not try to open the Ranger directly
- Use UDP on port `14550`
- Keep MAVProxy running the whole time; QGroundControl should attach to MAVProxy, not replace it
- If needed, close and reopen QGroundControl after MAVProxy is already up

## Scripts

| Script | Purpose |
|--------|---------|
| `video_capture.py` | Live drone camera feed, hardcoded to camera index 0. Press `q` to quit. |
| `main.py` | WASD manual flight control + HUD |
| `rocket_telemtry.py` | Passive telemetry monitor (altitude, GPS, orientation, battery) |
| `startup.py` | Main launcher for MAVProxy + telemetry + video |

## Current Status

Video capture is working again after rebooting the Walksnail system. During debugging, QuickTime also showed a black screen, which ruled out Python/OpenCV as the root cause of the temporary video failure.

The main blocker is now the telemetry/radio path. QGroundControl reported about `96%` packet loss over the radio link, and `startup.py` parameter download over that path was only about `1-3 params/sec` after the initial burst. The next diagnostic step is to connect the flight controller directly to the Mac over USB and compare QGroundControl loss rate and parameter download speed against the radio path.

## Build Order (remaining)

1. Telemetry pipeline — get altitude, GPS, heading reliably into Python
2. Mode controller — manage AUTO / GUIDED / MANUAL switching from one script
3. Vision → GPS math — pixel offset + altitude + FOV → target lat/lon
4. Close the loop — feed lat/lon into GUIDED mode commands
