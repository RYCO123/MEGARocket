"""
startup.py

Starts router.py (serial-to-UDP forwarder), confirms heartbeat, then launches
the ground station GUI and WASD flight control (RocketCommander).

RocketCommander is defined in main.py — not a package, just local code.

Usage:
    python3 startup.py
    python3 startup.py --video-source 1
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from main import RocketCommander
from pymavlink import mavutil
from serial.tools import list_ports

PREFERRED_SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD              = 57600
UDP_QGC           = '127.0.0.1:14550'
UDP_TELEMETRY     = '127.0.0.1:14551'
UDP_CONTROL       = '127.0.0.1:14552'
UDP_MONITOR       = '127.0.0.1:14553'
BASE_DIR          = Path(__file__).resolve().parent
CALIBRATION_CSV   = BASE_DIR / 'vision' / 'results' / 'calibration.csv'
ROUTER_SCRIPT     = BASE_DIR / 'router.py'
GROUND_STATION_SCRIPT = BASE_DIR / 'ground_station.py'
VIDEO_SOURCE_FILE = BASE_DIR / 'camera_index.txt'
HEARTBEAT_TIMEOUT = 12
ROUTER_LOCAL_PORTS = (14540, 14541, 14542)


# ---------------------------------------------------------------------------
# Video source
# ---------------------------------------------------------------------------

def read_video_source():
    if not VIDEO_SOURCE_FILE.exists():
        return '0'
    return VIDEO_SOURCE_FILE.read_text(encoding='utf-8').strip() or '0'


def write_video_source(source):
    VIDEO_SOURCE_FILE.write_text(f"{source}\n", encoding='utf-8')


def prompt_for_video_source(current):
    try:
        entered = input(f"Video source [{current}]: ").strip()
    except EOFError:
        return current, False
    if not entered:
        return current, False
    return entered, entered != current


# ---------------------------------------------------------------------------
# Serial port detection
# ---------------------------------------------------------------------------

def detect_serial_port():
    port_infos = list(list_ports.comports())
    if not port_infos:
        return None

    ports = sorted(p.device for p in port_infos)
    if PREFERRED_SERIAL_PORT in ports:
        return PREFERRED_SERIAL_PORT

    for prefix in ('/dev/cu.usbserial', '/dev/cu.usbmodem',
                   '/dev/cu.SLAB_USBtoUART', '/dev/cu.wchusbserial'):
        for port in ports:
            if port.startswith(prefix):
                return port

    usb_tokens = ('usb', 'uart', 'cp210', 'ftdi', 'ch340', 'silicon labs')
    for p in port_infos:
        identity = ' '.join(str(x or '') for x in
                            (p.device, p.description, p.hwid,
                             p.manufacturer, p.product)).lower()
        if any(t in identity for t in usb_tokens):
            return p.device
    return None


# ---------------------------------------------------------------------------
# Router (serial-to-UDP forwarder)
# ---------------------------------------------------------------------------

def start_router(serial_port):
    cmd = [sys.executable, str(ROUTER_SCRIPT), serial_port, str(BAUD)]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1, start_new_session=True)


def wait_for_heartbeat():
    time.sleep(0.3)   # let router bind UDP ports
    conn = mavutil.mavlink_connection(f'udp:{UDP_TELEMETRY}')
    print("Waiting for heartbeat...", end=' ', flush=True)
    hb = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    conn.close()
    if hb is None:
        raise TimeoutError(f"No heartbeat on UDP {UDP_TELEMETRY}.")
    print("OK")


def connect(serial_port):
    print(f"Starting router on {serial_port} @ {BAUD} baud...")
    proc = start_router(serial_port)

    try:
        time.sleep(0.3)
        code = proc.poll()
        if code is not None:
            err = proc.stdout.read()
            raise RuntimeError(
                f"router.py exited (code {code}). "
                f"{err or 'Port may be in use.'}"
            )

        wait_for_heartbeat()
        return proc
    except Exception:
        stop_process(proc, "router")
        raise


# ---------------------------------------------------------------------------
# Subprocess helpers
# ---------------------------------------------------------------------------

def start_script(path, *args):
    return subprocess.Popen([sys.executable, str(path), *args],
                            cwd=BASE_DIR, start_new_session=True)


def _signal_process_group(proc, sig):
    try:
        os.killpg(proc.pid, sig)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return False


def cleanup_stale_router_processes():
    lsof_cmd = ['lsof', '-t', '-nP']
    for port in ROUTER_LOCAL_PORTS:
        lsof_cmd.append(f'-iUDP:{port}')

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


def stop_process(proc, name):
    if proc.poll() is not None:
        return
    print(f"Stopping {name}...")
    signaled = _signal_process_group(proc, signal.SIGTERM)
    if not signaled:
        proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        if not _signal_process_group(proc, signal.SIGKILL):
            proc.kill()
        proc.wait(timeout=5)


def shutdown(processes):
    for name, proc in reversed(processes):
        stop_process(proc, name)


def monitor(processes):
    while True:
        for name, proc in processes:
            if proc.poll() is not None:
                raise RuntimeError(f"{name} exited unexpectedly.")
        time.sleep(1)


def monitor_in_background(processes):
    def _worker():
        try:
            monitor(processes)
        except Exception as exc:
            print(f"\nProcess failure: {exc}")
            signal.raise_signal(signal.SIGINT)
    threading.Thread(target=_worker, daemon=True).start()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="Launch router, ground station, and WASD flight control.")
    p.add_argument('--video-source',
                   help="Camera index or stream URL (e.g. 0, 1, rtsp://...)")
    p.add_argument('--save-video-source', action='store_true',
                   help="Persist --video-source to camera_index.txt")
    return p.parse_args()


if __name__ == '__main__':
    args = parse_args()
    processes = []
    try:
        # Video source
        saved = read_video_source()
        video_source = args.video_source or saved
        if args.video_source:
            source_changed = args.video_source != saved
        elif sys.stdin.isatty():
            video_source, source_changed = prompt_for_video_source(saved)
        else:
            source_changed = False
        if args.save_video_source or source_changed:
            write_video_source(video_source)
            print(f"Video source saved: {video_source}")

        # Serial port
        port = detect_serial_port()
        if not port:
            raise RuntimeError(
                "No serial device found. "
                "Plug in the radio adapter and verify with `ls /dev/cu.*`"
            )

        # Router + heartbeat
        cleanup_stale_router_processes()
        router_proc = connect(port)
        processes.append(("router", router_proc))

        if not CALIBRATION_CSV.exists():
            print(f"[vision] WARNING: lens calibration not found — projection disabled.")
            print(f"         Run:  python3 vision/calibrate_lens.py --height-cm <HEIGHT>\n")

        print(f"\nReady — {port} @ {BAUD}")
        print(f"  QGroundControl → udp:{UDP_QGC}")
        print(f"  Ground station → udp:{UDP_TELEMETRY}")
        print(f"  Flight control → udp:{UDP_CONTROL}")
        print(f"  Diagnostics    → udp:{UDP_MONITOR}")
        print(f"  Video source   → {video_source}\n")

        # Ground station GUI (vision + auto-launches QGC)
        processes.append(("ground_station",
                          start_script(GROUND_STATION_SCRIPT, '--source', video_source)))

        monitor_in_background(processes)

        print("Press Ctrl+C to stop.\n")

        # WASD flight control runs in the main thread (needs keyboard focus)
        commander = RocketCommander(connection_string=f'udp:{UDP_CONTROL}')
        commander.run()

    except KeyboardInterrupt:
        print("\nShutdown requested.")
    except Exception as exc:
        print(f"\nStartup failed: {exc}")
    finally:
        shutdown(processes)
        print("Done.")
