"""
mavlink_interface/param_manager.py

Pushes non-default parameters from config/params.yaml to the flight
controller over serial.

Calibration parameters are never written — see calibration/protected_params.py.

Run INSTEAD OF startup.py (connects directly to serial).
Re-run any time you edit config/params.yaml.

Usage:
    python3 mavlink_interface/param_manager.py
    python3 mavlink_interface/param_manager.py --verify-only
"""

import argparse
import sys
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

# Add project root to path so calibration module is importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from calibration.protected_params import is_protected

SERIAL_PORT       = '/dev/cu.usbserial-0001'
BAUD              = 57600
HEARTBEAT_TIMEOUT = 12
PARAM_ACK_TIMEOUT = 8    # generous — slow radio link drops packets
PARAM_RETRIES     = 3    # retry on timeout before giving up

PARAMS_FILE = Path(__file__).resolve().parent.parent / 'config' / 'params.yaml'


# ---------------------------------------------------------------------------
# MAVLink helpers
# ---------------------------------------------------------------------------

def connect():
    print(f"Connecting to {SERIAL_PORT} at {BAUD} baud ...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        raise TimeoutError(
            "No heartbeat. Is startup.py still running? Close it first."
        )
    print(f"  Connected — system {master.target_system}, "
          f"component {master.target_component}\n")
    return master


def set_param(master, name, value):
    for attempt in range(PARAM_RETRIES):
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + PARAM_ACK_TIMEOUT
        while time.time() < deadline:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if msg and msg.param_id.rstrip('\x00') == name:
                return float(msg.param_value)
    raise TimeoutError(f"No ACK for {name} after {PARAM_RETRIES} attempts")


def get_param(master, name):
    for attempt in range(PARAM_RETRIES):
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            -1,
        )
        deadline = time.time() + PARAM_ACK_TIMEOUT
        while time.time() < deadline:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
            if msg and msg.param_id.rstrip('\x00') == name:
                return float(msg.param_value)
    raise TimeoutError(f"Could not read {name} after {PARAM_RETRIES} attempts")


# ---------------------------------------------------------------------------
# Load and validate params
# ---------------------------------------------------------------------------

def load_params(path):
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)

    # Flatten — YAML is a flat dict of PARAM_NAME: value
    params = {}
    for key, value in raw.items():
        if isinstance(value, dict):
            # Shouldn't happen with flat YAML but handle gracefully
            for subkey, subval in value.items():
                params[subkey] = subval
        else:
            params[str(key)] = value
    return params


def filter_protected(params):
    """Remove any protected calibration params and warn."""
    clean = {}
    blocked = []
    for name, value in params.items():
        if is_protected(name):
            blocked.append(name)
        else:
            clean[name] = value
    if blocked:
        print(f"  Skipping {len(blocked)} protected calibration param(s):")
        for name in blocked:
            print(f"    {name}")
        print()
    return clean


# ---------------------------------------------------------------------------
# Push
# ---------------------------------------------------------------------------

def push_params(master, params):
    ok, failed = [], []

    for name, target_value in params.items():
        try:
            actual = set_param(master, name, target_value)
            ok.append((name, target_value, actual))
            match = abs(actual - float(target_value)) < 0.01
            status = 'OK' if match else f'MISMATCH got {actual}'
            print(f"  {name:<28} {float(target_value):>10.3f}  {status}")
        except TimeoutError as e:
            failed.append((name, str(e)))
            print(f"  {name:<28}  TIMEOUT")

    return ok, failed


def verify_params(master, params):
    mismatched, failed = [], []

    for name, target_value in params.items():
        try:
            actual = get_param(master, name)
            match = abs(actual - float(target_value)) < 0.01
            status = 'OK' if match else f'MISMATCH (FC={actual:.3f} want={float(target_value):.3f})'
            print(f"  {name:<28}  {status}")
            if not match:
                mismatched.append(name)
        except TimeoutError:
            print(f"  {name:<28}  TIMEOUT")
            failed.append(name)

    return mismatched, failed


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--verify-only', action='store_true',
                        help='Read params from FC and verify without writing')
    args = parser.parse_args()

    print()
    print("=" * 55)
    print("  PARAMETER MANAGER")
    print("=" * 55)

    params = load_params(PARAMS_FILE)
    params = filter_protected(params)

    print(f"Loaded {len(params)} parameters from config/params.yaml\n")

    master = connect()

    if args.verify_only:
        print("Verifying parameters (read-only)...\n")
        mismatched, failed = verify_params(master, params)
        print()
        if not mismatched and not failed:
            print("All parameters match.")
        else:
            if mismatched:
                print(f"Mismatched: {mismatched}")
            if failed:
                print(f"Failed to read: {failed}")
        return

    print("Pushing parameters...\n")
    ok, failed = push_params(master, params)

    print()
    print("=" * 55)
    print(f"  Done — {len(ok)} pushed, {len(failed)} failed")
    print("=" * 55)

    if failed:
        print("\nFailed params:")
        for name, err in failed:
            print(f"  {name}: {err}")

    print("""
Parameters are stored in FC flash and survive power cycles.
Re-run any time you edit config/params.yaml.

After pushing:
  - Check SERVO*_REVERSED in QGC — verify each surface deflects
    in the correct direction before arming.
  - TECS_SINK_MIN: tune from actual flight glide rate.
""")


if __name__ == '__main__':
    main()
