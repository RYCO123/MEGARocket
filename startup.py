"""
startup.py

Starts MAVProxy, waits for heartbeat, fetches the vehicle parameter list,
then launches telemetry and video scripts.

Usage:
    python3 startup.py
    python3 startup.py --video-source 1
    python3 startup.py --video-source rtsp://127.0.0.1:8554/live
"""

import argparse
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from main import RocketCommander
from pymavlink import mavutil
from serial.tools import list_ports
from tqdm import tqdm

PREFERRED_SERIAL_PORT = '/dev/cu.usbserial-0001'
BAUD_CANDIDATES = (57600, 115200, 460800)
UDP_QGC     = 'udp:127.0.0.1:14550'
UDP_TELEMETRY = 'udp:127.0.0.1:14551'
UDP_CONTROL = 'udp:127.0.0.1:14552'
BASE_DIR    = Path(__file__).resolve().parent
TELEMETRY_SCRIPT = BASE_DIR / 'rocket_telemtry.py'
VIDEO_SCRIPT = BASE_DIR / 'video_capture.py'
VIDEO_SOURCE_FILE = BASE_DIR / 'camera_index.txt'
HEARTBEAT_TIMEOUT_S = 12
PARAM_LIST_TIMEOUT_S = 10
PARAM_PROGRESS_TIMEOUT_S = 5


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch MAVProxy, telemetry, mission control, and video capture."
    )
    parser.add_argument(
        '--video-source',
        help=(
            "Camera index or stream/device source to use for video capture. "
            "Examples: 0, 1, /dev/video0, rtsp://..."
        ),
    )
    parser.add_argument(
        '--save-video-source',
        action='store_true',
        help="Persist the provided --video-source into camera_index.txt for future runs.",
    )
    return parser.parse_args()


def read_video_source():
    if not VIDEO_SOURCE_FILE.exists():
        return '0'
    source = VIDEO_SOURCE_FILE.read_text(encoding='utf-8').strip()
    return source or '0'


def write_video_source(source):
    VIDEO_SOURCE_FILE.write_text(f"{source}\n", encoding='utf-8')


def prompt_for_video_source(current_source):
    prompt = (
        f"Video source [{current_source}] "
        "(type a camera number like 0 or 1, or a stream URL/path): "
    )
    try:
        entered = input(prompt).strip()
    except EOFError:
        return current_source, False

    if not entered:
        return current_source, False
    return entered, entered != current_source


def detect_serial_port():
    port_infos = list(list_ports.comports())
    if not port_infos:
        return None

    ports = sorted(p.device for p in port_infos)
    if PREFERRED_SERIAL_PORT in ports:
        return PREFERRED_SERIAL_PORT

    preferred_prefixes = (
        '/dev/cu.usbserial',
        '/dev/cu.usbmodem',
        '/dev/cu.SLAB_USBtoUART',
        '/dev/cu.wchusbserial',
    )
    for prefix in preferred_prefixes:
        for port in ports:
            if port.startswith(prefix):
                return port

    # Fallback: only choose likely USB/UART adapters.
    usb_like = []
    for p in port_infos:
        identity = " ".join(
            str(x or "")
            for x in (p.device, p.description, p.hwid, p.manufacturer, p.product)
        ).lower()
        if any(
            token in identity
            for token in (
                "usb",
                "uart",
                "cp210",
                "ftdi",
                "ch340",
                "silicon labs",
                "vid:pid",
            )
        ):
            usb_like.append(p.device)
    if usb_like:
        return sorted(usb_like)[0]
    return None


def start_mavproxy(serial_port, baud):
    cmd = [
        sys.executable, '-m', 'MAVProxy.mavproxy',
        f'--master={serial_port}',
        f'--baudrate={baud}',
        '--streamrate=-1',
        f'--out={UDP_QGC}',
        f'--out={UDP_TELEMETRY}',
        f'--out={UDP_CONTROL}',
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    return proc


def start_script(script_path, *args):
    return subprocess.Popen([sys.executable, str(script_path), *args], cwd=BASE_DIR)


def wait_for_params():
    print("Connecting via MAVProxy...")

    # Give MAVProxy a moment to bind the UDP port
    time.sleep(2)

    conn = mavutil.mavlink_connection(UDP_TELEMETRY)

    print("Waiting for heartbeat...", end=' ', flush=True)
    hb = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    if hb is None:
        conn.close()
        raise TimeoutError("No heartbeat seen on UDP 14551.")
    print("OK")

    # Request only parameters instead of enabling all telemetry streams.
    conn.param_fetch_all()

    # Wait for first PARAM_VALUE to know total count
    print("Waiting for parameter list...")
    while True:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=PARAM_LIST_TIMEOUT_S)
        if msg:
            total = msg.param_count
            break

    seen = set()
    bar = tqdm(total=total, desc="Parameters", unit="param", ncols=70)

    while len(seen) < total:
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=PARAM_PROGRESS_TIMEOUT_S)
        if msg is None:
            break
        if msg.param_index not in seen:
            seen.add(msg.param_index)
            bar.update(1)

    bar.close()
    conn.close()


def stop_mavproxy(proc):
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            return


def try_connect_with_bauds(serial_port):
    last_error = None
    print(f"Connecting on serial port: {serial_port}")
    for baud in BAUD_CANDIDATES:
        print(f"Trying baud {baud}...")
        proc = start_mavproxy(serial_port, baud)
        try:
            # If MAVProxy exits immediately, usually serial port is busy/unavailable.
            time.sleep(1.2)
            code = proc.poll()
            if code is not None:
                err = proc.stderr.read().decode("utf-8", errors="ignore").strip()
                raise RuntimeError(
                    f"MAVProxy exited early (code {code}). "
                    f"{err or 'Serial port may be in use (close QGC serial auto-connect).'}"
                )
            wait_for_params()
            print(f"Connected at baud {baud}.")
            return proc, baud
        except Exception as exc:
            last_error = exc
            stop_mavproxy(proc)
            print(f"  Failed at {baud}: {exc}")
    raise RuntimeError(f"Unable to acquire heartbeat on {serial_port}. Last error: {last_error}")


def stop_process(proc, name):
    if proc.poll() is not None:
        return

    print(f"Stopping {name}...")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print(f"{name} did not exit after SIGTERM, killing it.")
        proc.kill()
        proc.wait(timeout=5)


def shutdown(processes):
    for name, proc in reversed(processes):
        stop_process(proc, name)


def monitor(processes):
    while True:
        for name, proc in processes:
            code = proc.poll()
            if code is not None:
                raise RuntimeError(f"{name} exited unexpectedly with code {code}.")
        time.sleep(1)


def monitor_in_background(processes):
    def _worker():
        try:
            monitor(processes)
        except Exception as exc:
            print(f"\nBackground process failure: {exc}")
            signal.raise_signal(signal.SIGINT)

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()
    return thread


if __name__ == "__main__":
    args = parse_args()
    processes = []
    try:
        saved_video_source = read_video_source()
        video_source = args.video_source or saved_video_source
        source_changed = False

        if args.video_source:
            source_changed = args.video_source != saved_video_source
        elif sys.stdin.isatty():
            video_source, source_changed = prompt_for_video_source(saved_video_source)

        if args.save_video_source or source_changed:
            write_video_source(video_source)
            print(f"Saved video source to {VIDEO_SOURCE_FILE.name}: {video_source}")

        active_serial_port = detect_serial_port()
        if not active_serial_port:
            raise RuntimeError(
                "No USB telemetry serial device found. "
                "Plug in the adapter and run `ls /dev/cu.*` to verify a /dev/cu.usb* device exists."
            )

        print(f"Using serial port: {active_serial_port}")
        mavproxy_proc, active_baud = try_connect_with_bauds(active_serial_port)
        processes.append(("MAVProxy", mavproxy_proc))

        print("\nSystem ready. MAVProxy is running on:")
        print(f"  Serial → {active_serial_port} @ {active_baud}")
        print(f"  QGroundControl → {UDP_QGC}")
        print(f"  Telemetry HUD → {UDP_TELEMETRY}")
        print(f"  Mission control → {UDP_CONTROL}")
        print(f"  Video source → {video_source}")

        print("\nStarting telemetry console...")
        telemetry_proc = start_script(TELEMETRY_SCRIPT)
        processes.append(("telemetry", telemetry_proc))

        print("Starting video capture...")
        video_proc = start_script(VIDEO_SCRIPT, '--source', video_source)
        processes.append(("video capture", video_proc))

        print("Starting mission control (WASD) in this terminal...")
        monitor_in_background(processes)

        print("\nPress Ctrl+C to stop all processes.\n")
        commander = RocketCommander(connection_string=UDP_CONTROL)
        commander.run()
    except KeyboardInterrupt:
        print("\nShutdown requested.")
    except Exception as exc:
        print(f"\nStartup failed: {exc}")
    finally:
        shutdown(processes)
        print("Done.")
