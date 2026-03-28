"""
startup.py

Starts MAVProxy silently, then shows a progress bar while ArduPilot
downloads its parameter list. Prints "Ready" when the system is online.

Usage:
    python3 startup.py

After this completes, run telemetry.py or any other script in a second terminal.
"""

import subprocess
import sys
import time
from pymavlink import mavutil
from tqdm import tqdm

SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD        = 57600  # change to 115200 after updating SERIAL0_BAUD in ArduPilot params
UDP_QGC     = 'udp:127.0.0.1:14550'
UDP_PYTHON  = 'udp:127.0.0.1:14551'


def start_mavproxy():
    cmd = [
        sys.executable, '-m', 'MAVProxy.mavproxy',
        f'--master={SERIAL_PORT}',
        f'--baudrate={BAUD}',
        f'--out={UDP_QGC}',
        f'--out={UDP_PYTHON}',
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc


def wait_for_params():
    print(f"Connecting to {SERIAL_PORT} at {BAUD} baud via MAVProxy...")

    # Give MAVProxy a moment to bind the UDP port
    time.sleep(2)

    conn = mavutil.mavlink_connection(UDP_PYTHON)

    print("Waiting for heartbeat...", end=' ', flush=True)
    conn.wait_heartbeat()
    print("OK")

    # Request all data streams so we start receiving PARAM_VALUE
    conn.mav.request_data_stream_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        4, 1
    )

    # Wait for first PARAM_VALUE to know total count
    print("Waiting for parameter list...")
    while True:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
        if msg:
            total = msg.param_count
            break

    seen = set()
    bar = tqdm(total=total, desc="Parameters", unit="param", ncols=70)

    while len(seen) < total:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if msg is None:
            break
        if msg.param_index not in seen:
            seen.add(msg.param_index)
            bar.update(1)

    bar.close()
    conn.close()


if __name__ == "__main__":
    proc = start_mavproxy()
    try:
        wait_for_params()
        print("\nSystem ready. MAVProxy is running on:")
        print(f"  QGroundControl → {UDP_QGC}")
        print(f"  Python scripts → {UDP_PYTHON}")
        print("\nPress Ctrl+C to shut down MAVProxy.\n")
        proc.wait()
    except KeyboardInterrupt:
        print("\nShutting down MAVProxy...")
        proc.terminate()
        proc.wait()
        print("Done.")
