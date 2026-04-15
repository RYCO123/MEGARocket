"""
mavlink_interface/servo_output_monitor.py

Display SERVO_OUTPUT_RAW from the flight controller so output commands can be
verified independently of downstream wiring, ESC boards, or servos.

Usage examples:
    python3 mavlink_interface/servo_output_monitor.py
    python3 mavlink_interface/servo_output_monitor.py --connect udp:127.0.0.1:14553
"""

import argparse
import os
import signal
import subprocess
import time

from pymavlink import mavutil

DEFAULT_SERIAL_PORT = "/dev/cu.usbserial-0001"
DEFAULT_BAUD = 57600
DEFAULT_UDP = "udp:127.0.0.1:14553"
HEARTBEAT_TIMEOUT = 12
ROUTER_LOCAL_PORTS = (14540, 14541, 14542)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Monitor MAVLink SERVO_OUTPUT_RAW values."
    )
    parser.add_argument(
        "--connect",
        default=DEFAULT_SERIAL_PORT,
        help=(
            "Connection string. Use the serial device when startup.py is not "
            "running, or udp:127.0.0.1:14553 when listening through router.py."
        ),
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help="Baud rate for serial connections.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=4.0,
        help="Requested SERVO_OUTPUT_RAW rate in Hz.",
    )
    return parser.parse_args()


def open_connection(connect_string, baud):
    if connect_string.startswith("udp:"):
        return mavutil.mavlink_connection(connect_string)
    return mavutil.mavlink_connection(connect_string, baud=baud)


def cleanup_stale_router_processes():
    lsof_cmd = ["lsof", "-t", "-nP"]
    for port in ROUTER_LOCAL_PORTS:
        lsof_cmd.append(f"-iUDP:{port}")

    result = subprocess.run(lsof_cmd, capture_output=True, text=True, check=False)
    if result.returncode not in (0, 1):
        return

    pids = sorted({int(line.strip()) for line in result.stdout.splitlines() if line.strip()})
    if not pids:
        return

    print(f"Cleaning up stale router listener(s): {', '.join(str(pid) for pid in pids)}")
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            continue

    deadline = time.time() + 2.0
    while time.time() < deadline:
        remaining = []
        for pid in pids:
            try:
                os.kill(pid, 0)
                remaining.append(pid)
            except ProcessLookupError:
                continue
        if not remaining:
            return
        time.sleep(0.1)

    for pid in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            continue


def request_stream(master, rate_hz):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        int(1_000_000 / rate_hz),
        0,
        0,
        0,
        0,
        0,
    )


def main():
    args = parse_args()
    if not args.connect.startswith("udp:"):
        cleanup_stale_router_processes()
    print(f"Connecting to {args.connect} ...")
    master = open_connection(args.connect, args.baud)
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        raise TimeoutError("No heartbeat received.")

    print(
        f"Connected to system {master.target_system}, component {master.target_component}"
    )
    request_stream(master, args.rate)
    print("Watching SERVO_OUTPUT_RAW. Press Ctrl+C to stop.")

    last_line = ""
    try:
        while True:
            msg = master.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=1.0)
            if msg is None:
                continue
            line = (
                f"S1={msg.servo1_raw:4} "
                f"S2={msg.servo2_raw:4} "
                f"S3={msg.servo3_raw:4} "
                f"S4={msg.servo4_raw:4} "
                f"S5={msg.servo5_raw:4} "
                f"S6={msg.servo6_raw:4} "
                f"S7={msg.servo7_raw:4} "
                f"S8={msg.servo8_raw:4}"
            )
            if line != last_line:
                print(line)
                last_line = line
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
