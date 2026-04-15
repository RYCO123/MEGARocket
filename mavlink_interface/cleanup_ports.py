"""
mavlink_interface/cleanup_ports.py

Terminate stale MAVLink-related Python processes that are holding the serial
device or local UDP fanout ports.

Usage:
    python3 mavlink_interface/cleanup_ports.py
"""

import os
import signal
import subprocess
import time

SERIAL_DEVICE = "/dev/cu.usbserial-0001"
UDP_PORTS = (14540, 14541, 14542, 14543, 14550, 14551, 14552, 14553)


def find_pids():
    cmd = ["lsof", "-t", "-nP", SERIAL_DEVICE]
    for port in UDP_PORTS:
        cmd.append(f"-iUDP:{port}")

    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.strip() or "lsof failed")

    return sorted({int(line.strip()) for line in result.stdout.splitlines() if line.strip()})


def signal_pids(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            continue


def remaining_pids(pids):
    alive = []
    for pid in pids:
        try:
            os.kill(pid, 0)
            alive.append(pid)
        except ProcessLookupError:
            continue
    return alive


def main():
    pids = find_pids()
    if not pids:
        print("No stale MAVLink-related processes found.")
        return

    print(f"Cleaning up PID(s): {', '.join(str(pid) for pid in pids)}")
    signal_pids(pids, signal.SIGTERM)

    deadline = time.time() + 2.0
    while time.time() < deadline:
        alive = remaining_pids(pids)
        if not alive:
            print("Cleanup complete.")
            return
        time.sleep(0.1)

    signal_pids(alive, signal.SIGKILL)
    time.sleep(0.2)
    alive = remaining_pids(alive)
    if alive:
        print(f"Still running after SIGKILL: {', '.join(str(pid) for pid in alive)}")
    else:
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
