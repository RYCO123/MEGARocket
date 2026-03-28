# Rocket/Drone Ground Station

## System Overview

Autonomous UAV ground station with computer vision targeting. The craft flies a default mission plan, the downward camera detects an object of interest, and the ground station dynamically redirects the craft to investigate — then returns it to the original mission.

## Data Flow

```
                         ┌── QGroundControl (UDP 14550)  — monitoring UI
Flight Controller ──── MAVProxy
   (ArduPilot)           └── Python scripts  (UDP 14551)  — vision + autonomy
```

**Telemetry IN:** MAVProxy receives from the craft and fans out to both QGC and Python simultaneously.

**Commands OUT:** Python sends directly to MAVProxy → craft. QGC sends directly to MAVProxy → craft. Neither goes through the other — they are peers.

## Flight Modes

| Mode | Trigger | Description |
|------|---------|-------------|
| AUTO | Default | Craft follows QGC mission plan autonomously |
| GUIDED | Vision detects target | Python sends GPS coordinates, craft redirects. Resumes AUTO when done. |
| MANUAL/QSTABILIZE | WASD override | Python sends RC overrides directly to MAVProxy |

GUIDED mode and WASD overrides are sent **Python → MAVProxy → craft**. They do not route through QGroundControl.

## Vision Pipeline (in progress)

1. Capture video from drone camera (`cv2.VideoCapture(0)`, 1920x1080 @ 10fps)
2. Process frames to detect object of interest
3. Convert pixel offset → GPS coordinates using telemetry (altitude, heading, camera FOV)
4. Switch craft to GUIDED mode
5. Send `SET_POSITION_TARGET_GLOBAL_INT` with target GPS coords (~1Hz)
6. Switch back to AUTO when done

## Setup

Find the USB port:
```bash
ls /dev/cu.usbmodem*
```

Start MAVProxy (must do this before opening QGC):
```bash
python3 -m MAVProxy.mavproxy --master=/dev/cu.usbmodem11201 --baudrate 115200 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Startup order:
1. Close QGroundControl
2. Run MAVProxy command above
3. Open QGroundControl
4. Run Python script in separate terminal

Activate virtual environment:
```bash
source venv/bin/activate
```

## Scripts

| Script | Purpose |
|--------|---------|
| `video_capture.py` | Live drone camera feed (index 0). Press `q` to quit. |
| `main.py` | WASD manual flight control + HUD |
| `rocket_telemtry.py` | Telemetry monitor (altitude, GPS, orientation, battery) |

## Build Order (remaining)

1. Telemetry pipeline — get altitude, GPS, heading reliably into Python
2. Mode controller — manage AUTO / GUIDED / MANUAL switching from one script
3. Vision → GPS math — pixel offset + altitude + FOV → target lat/lon
4. Close the loop — feed lat/lon into GUIDED mode commands
