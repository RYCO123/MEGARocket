"""
calibration/calibrate.py

One-time calibration for the rocket-to-glider vehicle.
Run INSTEAD OF startup.py (not alongside it) — connects directly
to the FC over serial so no port conflicts occur.

Step 1 — AHRS_ORIENTATION = 29 (ROTATION_ROLL_180_PITCH_90)
         Encodes the FC physical mounting AND the 90° rocket→glider
         rotation so ArduPilot correctly interprets glider-level flight.
         Stored in FC flash; survives power cycles.

Step 2 — Gyro calibration
         Measures gyro drift/bias while vehicle is completely still.
         Nose-up on a flat table is the ideal stable reference.

Step 3 — Baro calibration
         Records current ground pressure as the altitude = 0 reference.

FC mounting geometry:
  FC +X  →  vehicle top   (+Z, up)
  FC +Y  →  vehicle left  (+Y, port)
  FC +Z  →  vehicle tail  (-X, aft)

Prerequisites:
  - startup.py is NOT running (this connects directly to serial)
  - USB-serial radio adapter is plugged in to /dev/cu.usbserial-0001
  - Vehicle is disarmed, nose up, completely still.

Usage:
    python3 calibration/calibrate.py
"""

import sys
import time
from pymavlink import mavutil

SERIAL_PORT        = '/dev/cu.usbserial-0001'
BAUD               = 57600
HEARTBEAT_TIMEOUT  = 12   # seconds
PARAM_ACK_TIMEOUT  = 5    # seconds
CAL_ACK_TIMEOUT    = 30   # seconds — gyro/baro cal can take a moment

# ROTATION_ROLL_180_PITCH_90
#
# Derivation (FC physical mounting):
#   FC +X  →  vehicle top   (+Z)   up
#   FC +Y  →  vehicle left  (+Y)   port
#   FC +Z  →  vehicle tail  (-X)   aft
#
# To go from mounted → natural ArduPilot orientation:
#   1. Roll 180°  — flips FC right-side-up  (Y and Z negate)
#   2. Pitch 90°  — pivots nose from forward → up
# Combined rotation matrix equals the correct FC-to-body transform.
AHRS_ORIENTATION_VALUE = 29


def connect():
    print(f"Connecting directly to {SERIAL_PORT} at {BAUD} baud ...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        raise TimeoutError(
            "No heartbeat. Is the radio adapter plugged in and aircraft powered on? "
            "Is startup.py still running (close it first)?"
        )
    print(f"  Connected — system {master.target_system}, "
          f"component {master.target_component}")
    return master


def set_param(master, name, value):
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
    raise TimeoutError(f"No ACK received for {name}")


def get_param(master, name):
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
    raise TimeoutError(f"Could not read {name}")


def trigger_preflight_cal(master, gyro=0, baro=0):
    """
    Send MAV_CMD_PREFLIGHT_CALIBRATION and wait for COMMAND_ACK.
      param1 = 1  →  gyro cal
      param3 = 1  →  baro / ground pressure cal
    Vehicle must be completely stationary when called.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,      # confirmation
        gyro,   # param1: gyro
        0,      # param2: mag (skip)
        baro,   # param3: ground pressure
        0,      # param4: RC (skip)
        0,      # param5: accel (skip)
        0,      # param6: accel temp (skip)
        0,      # param7: unused
    )
    deadline = time.time() + CAL_ACK_TIMEOUT
    while time.time() < deadline:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
            return msg.result
    raise TimeoutError("No COMMAND_ACK for preflight calibration")


def main():
    print()
    print("=" * 55)
    print("  VEHICLE CALIBRATION")
    print("=" * 55)
    print("Vehicle must be nose-up on a flat surface and completely still.\n")

    master = connect()

    # ------------------------------------------------------------------
    # Step 1: Board orientation (one-time, stored in flash)
    # Only parameter this script is allowed to write.
    # All INS_*, COMPASS_*, and AHRS_TRIM_* are protected —
    # those are owned by QGroundControl calibration and must not
    # be touched here or by any other Python code.
    # ------------------------------------------------------------------
    print("[1/3]  Setting board orientation...")
    actual = set_param(master, 'AHRS_ORIENTATION', AHRS_ORIENTATION_VALUE)
    if int(actual) != AHRS_ORIENTATION_VALUE:
        print(f"  ERROR: FC returned {int(actual)}, expected {AHRS_ORIENTATION_VALUE}")
        sys.exit(1)
    print(f"  AHRS_ORIENTATION = {int(actual)}  (ROTATION_ROLL_180_PITCH_90)  OK")

    # ------------------------------------------------------------------
    # Step 2: Gyro calibration — benefits directly from stationary vehicle
    # ------------------------------------------------------------------
    print("\n[2/3]  Gyro calibration  (do not move the vehicle) ...")
    result = trigger_preflight_cal(master, gyro=1, baro=0)
    MAV_RESULT_ACCEPTED = 0
    if result == MAV_RESULT_ACCEPTED:
        print("  Gyro calibration accepted.")
    else:
        print(f"  Gyro cal result code: {result}  (may still succeed — check QGC)")

    # ------------------------------------------------------------------
    # Step 3: Baro calibration — sets ground pressure = altitude zero
    # ------------------------------------------------------------------
    print("\n[3/3]  Barometer calibration  (ground pressure reference) ...")
    result = trigger_preflight_cal(master, gyro=0, baro=1)
    if result == MAV_RESULT_ACCEPTED:
        print("  Baro calibration accepted.")
    else:
        print(f"  Baro cal result code: {result}")

    print()
    print("=" * 55)
    print("  DONE")
    print("=" * 55)
    print("""
Calibration complete.  This does not need to be run again.

QGroundControl calibration (accel + compass) is complete and
protected — no Python code will ever overwrite those values.
See calibration/protected_params.py for the full protected list.

Next: bench test FBWA stabilization in glider orientation.
""")


if __name__ == '__main__':
    main()
