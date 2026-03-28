"""
telemetry.py

Connects to MAVProxy on UDP 14551 and prints a clean telemetry readout every second.
MAVProxy must be running before starting this script.

Usage:
    python3 telemetry.py
"""

import time
import threading
from pymavlink import mavutil

MAVPROXY_UDP = 'udp:127.0.0.1:14551'

state = {}
state_lock = threading.Lock()


def listener():
    conn = mavutil.mavlink_connection(MAVPROXY_UDP)
    print(f"Waiting for heartbeat on {MAVPROXY_UDP} ...")
    conn.wait_heartbeat()
    print("Heartbeat received — streaming telemetry\n")

    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10, 1
    )

    while True:
        msg = conn.recv_match(blocking=True)
        if msg is None or msg.get_type() == 'BAD_DATA':
            continue
        with state_lock:
            state[msg.get_type()] = msg.to_dict()


def get(msg_type, *keys):
    d = state.get(msg_type, {})
    return [d.get(k) for k in keys]


def print_telemetry():
    with state_lock:
        snap = dict(state)

    if not snap:
        print("No data yet...")
        return

    print(f"\n{'='*50}")
    print(f"  {time.strftime('%H:%M:%S')}")
    print(f"{'='*50}")

    # --- Position ---
    gps = snap.get('GLOBAL_POSITION_INT', {})
    if gps:
        lat = gps.get('lat', 0) / 1e7
        lon = gps.get('lon', 0) / 1e7
        alt_msl = gps.get('alt', 0) / 1000
        alt_rel = gps.get('relative_alt', 0) / 1000
        hdg = gps.get('hdg', 0) / 100
        print(f"  Position   lat={lat:.6f}  lon={lon:.6f}")
        print(f"  Altitude   {alt_rel:.1f}m AGL  /  {alt_msl:.1f}m MSL")
        print(f"  Heading    {hdg:.1f}°")

    # --- Attitude ---
    att = snap.get('ATTITUDE', {})
    if att:
        import math
        pitch = math.degrees(att.get('pitch', 0))
        roll  = math.degrees(att.get('roll', 0))
        yaw   = math.degrees(att.get('yaw', 0))
        print(f"  Attitude   pitch={pitch:.1f}°  roll={roll:.1f}°  yaw={yaw:.1f}°")

    # --- Speed ---
    hud = snap.get('VFR_HUD', {})
    if hud:
        airspeed   = hud.get('airspeed', 0)
        groundspeed = hud.get('groundspeed', 0)
        climb      = hud.get('climb', 0)
        throttle   = hud.get('throttle', 0)
        print(f"  Speed      airspeed={airspeed:.1f}m/s  groundspeed={groundspeed:.1f}m/s  climb={climb:.1f}m/s")
        print(f"  Throttle   {throttle}%")

    # --- GPS quality ---
    gps_raw = snap.get('GPS_RAW_INT', {})
    if gps_raw:
        fix  = gps_raw.get('fix_type', 0)
        sats = gps_raw.get('satellites_visible', 0)
        fix_str = {0:'No GPS', 1:'No fix', 2:'2D', 3:'3D', 4:'DGPS', 5:'RTK float', 6:'RTK fixed'}.get(fix, str(fix))
        print(f"  GPS        fix={fix_str}  sats={sats}")

    # --- Battery ---
    sys = snap.get('SYS_STATUS', {})
    if sys:
        volt = sys.get('voltage_battery', 0) / 1000
        curr = sys.get('current_battery', -1)
        rem  = sys.get('battery_remaining', -1)
        curr_str = f"{curr/100:.1f}A" if curr >= 0 else "N/A"
        rem_str  = f"{rem}%" if rem >= 0 else "N/A"
        print(f"  Battery    {volt:.2f}V  current={curr_str}  remaining={rem_str}")

    # --- Flight mode & arming ---
    hb = snap.get('HEARTBEAT', {})
    if hb:
        armed  = bool(hb.get('base_mode', 0) & 128)
        mode   = hb.get('custom_mode', '?')
        print(f"  Mode       {mode}  |  {'ARMED' if armed else 'DISARMED'}")

    # --- Status messages from autopilot ---
    st = snap.get('STATUSTEXT', {})
    if st:
        print(f"  AP msg     {st.get('text', '').strip()}")


if __name__ == "__main__":
    t = threading.Thread(target=listener, daemon=True)
    t.start()

    try:
        while True:
            time.sleep(1)
            print_telemetry()
    except KeyboardInterrupt:
        print("\nStopped.")
