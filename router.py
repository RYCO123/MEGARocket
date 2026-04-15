"""
router.py

Minimal MAVLink serial-to-UDP router. Replaces MAVProxy for pure forwarding.
No plugin system, no logging, no parameter caching — just byte forwarding.

Serial data is broadcast to all UDP endpoints.
UDP data from any endpoint is written to serial.

Each router socket binds to a local port (14540-14542) and sends to the
target endpoint port (14550-14552). The endpoint app records our source
address and replies to it — completing the bidirectional link.

Usage (normally launched by startup.py):
    python3 router.py /dev/cu.usbserial-0001 57600
"""

import socket
import sys
import threading

import serial

# (local_bind_port, target_host, target_port)
ENDPOINTS = [
    (14540, '127.0.0.1', 14550),  # QGroundControl
    (14541, '127.0.0.1', 14551),  # Python ground station / telemetry
    (14542, '127.0.0.1', 14552),  # WASD flight control (RocketCommander in main.py)
    (14543, '127.0.0.1', 14553),  # Passive monitors / diagnostics
]

UDP_BUF = 4096


def _udp_to_serial(sock: socket.socket, ser: serial.Serial) -> None:
    """Forward UDP packets received on a router socket to serial."""
    sock.settimeout(1.0)
    while True:
        try:
            data, _ = sock.recvfrom(UDP_BUF)
            if data:
                ser.write(data)
        except socket.timeout:
            continue
        except OSError:
            break


def _serial_to_udp(ser: serial.Serial, socks: list) -> None:
    """Forward serial bytes to all UDP endpoint sockets."""
    while True:
        try:
            # Block on first byte, then drain the buffer in one shot.
            chunk = ser.read(1)
            pending = ser.in_waiting
            if pending:
                chunk += ser.read(pending)
            for sock, target in socks:
                sock.sendto(chunk, target)
        except serial.SerialException:
            break


def main() -> None:
    if len(sys.argv) < 3:
        sys.exit(f"Usage: {sys.argv[0]} DEVICE BAUD")

    device = sys.argv[1]
    baud   = int(sys.argv[2])
    ser = None
    socks = []
    try:
        ser = serial.Serial(device, baud)
        print(f"[router] {device} @ {baud} baud")

        for local_port, host, port in ENDPOINTS:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', local_port))
            target = (host, port)
            socks.append((sock, target))
            threading.Thread(target=_udp_to_serial, args=(sock, ser),
                             daemon=True).start()
            print(f"[router] :{local_port} → {host}:{port}")

        print("[router] running — Ctrl+C to stop")
        _serial_to_udp(ser, socks)
    except KeyboardInterrupt:
        print("\n[router] stopped")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
        for sock, _ in socks:
            sock.close()


if __name__ == "__main__":
    main()
