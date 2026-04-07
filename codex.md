# codex.md

## Current State (Apr 2026)

- `startup.py` is the main launcher.
- `startup.py` starts MAVProxy with `--streamrate=-1`.
- `startup.py` fetches parameters with `param_fetch_all()`.
- `startup.py` applies a lean telemetry profile to both `SR1_*` and `SR2_*`:
  - `POSITION = 2`
  - `EXT_STAT = 1`
  - `EXTRA1 = 5`
  - `EXTRA2 = 2`
  - `RAW_SENS = 0`
  - `RC_CHAN = 0`
  - `RAW_CTRL = 0`
- `startup.py` then launches `rocket_telemtry.py` and `video_capture.py`.

## Telemetry Notes

- `rocket_telemtry.py` is passive now.
- It does not send `REQUEST_DATA_STREAM`.
- It does not send `MAV_CMD_SET_MESSAGE_INTERVAL`.
- QGroundControl reported about `96%` loss rate over the radio link.
- Parameter download over the radio path was only about `1-3 params/sec` after the initial burst.
- Next diagnostic step: connect the flight controller directly to the Mac over USB and compare QGC loss rate and parameter download speed.

## Video Notes

- `video_capture.py` is hardcoded to camera index `0`.
- Index `1` is the MacBook camera.
- Index `2` is the iPhone continuity camera.
- A temporary black-screen problem was traced to the Walksnail chain rather than Python/OpenCV.
- QuickTime also showed black during the failure.
- Rebooting the Walksnail system restored video.
