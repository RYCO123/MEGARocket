import time
from pymavlink import mavutil
import serial.tools.list_ports

MONITORED_MSG_TYPES = (
    "GLOBAL_POSITION_INT",
    "ATTITUDE",
    "VFR_HUD",
    "SYS_STATUS",
)

MESSAGE_INTERVAL_US = {
    "GLOBAL_POSITION_INT": 500000,  # 2 Hz
    "ATTITUDE": 200000,             # 5 Hz
    "VFR_HUD": 500000,              # 2 Hz
    "SYS_STATUS": 1000000,          # 1 Hz
}
HUD_INTERVAL_S = 0.25
STALE_WARNING_S = 5.0

def find_ranger_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbserial" in port.device or "usbmodem" in port.device:
            return port.device
    return None

def render_hud(data, packet_count, window_s):
    """Renders a compact in-place telemetry dashboard."""
    now = time.time()
    rate_parts = []
    age_parts = []

    for msg_type in MONITORED_MSG_TYPES:
        short_name = {
            "ATTITUDE": "ATT",
            "GLOBAL_POSITION_INT": "GPS",
            "VFR_HUD": "VFR",
            "SYS_STATUS": "SYS",
        }[msg_type]
        hz = data["msg_counts"][msg_type] / window_s if window_s > 0 else 0.0
        rate_parts.append(f"{short_name}:{hz:>4.1f}/s")

        last_seen = data["last_seen"][msg_type]
        age_s = now - last_seen if last_seen else float("inf")
        age_label = "n/a" if last_seen == 0 else f"{age_s:.1f}s"
        age_parts.append(f"{short_name}:{age_label}")

    packets_per_second = packet_count / window_s if window_s > 0 else 0.0
    stale_msgs = [
        msg_type for msg_type in MONITORED_MSG_TYPES
        if data["last_seen"][msg_type] == 0 or now - data["last_seen"][msg_type] > STALE_WARNING_S
    ]
    status_line = (
        f"Link Warning: stale {'/'.join(stale_msgs)}"
        if stale_msgs else
        f"Last Other Msg: {data['last_other_msg']}"
    )

    lines = [
        "--- ROCKET DATA ---",
        f"Battery: {data['batt']:.2f}V",
        f"Altitude: {data['alt']:.1f}m | Climb: {data['v_speed']:+.1f}m/s",
        f"Pitch: {data['pitch']:+.1f}° | Roll: {data['roll']:+.1f}°",
        f"GPS: {data['lat']:.6f}, {data['lon']:.6f}",
        f"Packets Rx/s: {packets_per_second:.1f}",
        f"Msg Rates: {' | '.join(rate_parts)}",
        f"Msg Age:   {' | '.join(age_parts)}",
        status_line,
        "",
        "Press Ctrl+C to stop.",
    ]
    print("\x1b[H\x1b[J" + "\n".join(lines), end="", flush=True)

def start_telemetry():
    print("Connecting to MAVProxy on UDP 14551...")
    connection = mavutil.mavlink_connection('udp:127.0.0.1:14551')

    print("Waiting for Rocket Heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received! Link is ACTIVE.")
    print("Passive mode: using telemetry streams already configured on the vehicle/QGC.")

    data = {
        "alt": 0,
        "lat": 0,
        "lon": 0,
        "pitch": 0,
        "roll": 0,
        "v_speed": 0,
        "batt": 0,
        "msg_counts": {msg_type: 0 for msg_type in MONITORED_MSG_TYPES},
        "last_seen": {msg_type: 0.0 for msg_type in MONITORED_MSG_TYPES},
        "last_other_msg": "n/a",
    }
    packet_count = 0
    last_render = time.time()

    try:
        while True:
            # Drain the entire buffer so we always have the latest values
            while True:
                msg = connection.recv_match(blocking=False)
                if msg is None:
                    break
                packet_count += 1
                msg_type = msg.get_type()

                if msg_type in data["msg_counts"]:
                    data["msg_counts"][msg_type] += 1
                    data["last_seen"][msg_type] = time.time()
                else:
                    data["last_other_msg"] = msg_type

                # Check for GPS and Altitude
                if msg_type == 'GLOBAL_POSITION_INT':
                    data["alt"] = msg.relative_alt / 1000.0
                    data["lat"] = msg.lat / 1e7
                    data["lon"] = msg.lon / 1e7

                # Check for Orientation
                elif msg_type == 'ATTITUDE':
                    data["pitch"] = msg.pitch * (180/3.14159)
                    data["roll"] = msg.roll * (180/3.14159)

                # Check for Battery and System Status
                elif msg_type == 'SYS_STATUS':
                    data["batt"] = msg.voltage_battery / 1000.0

                # Check for Vertical Speed
                elif msg_type == 'VFR_HUD':
                    data["v_speed"] = msg.climb

            time.sleep(0.001)

            now = time.time()
            window_s = now - last_render
            if window_s >= HUD_INTERVAL_S:
                render_hud(data, packet_count, window_s)
                packet_count = 0
                for msg_type in data["msg_counts"]:
                    data["msg_counts"][msg_type] = 0
                last_render = now

    except KeyboardInterrupt:
        print("\nStopping...")
        connection.close()

if __name__ == "__main__":
    start_telemetry()
