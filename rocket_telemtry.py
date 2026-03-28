import time
from pymavlink import mavutil
import serial.tools.list_ports

def find_ranger_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usbserial" in port.device or "usbmodem" in port.device:
            return port.device
    return None

def request_data_streams(conn):
    """Sends a request to ArduPilot to start streaming telemetry data."""
    print("Requesting data streams from Flight Controller...")
    # This command asks for ALL data streams at 10Hz
    conn.mav.request_data_stream_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 
        10, # Rate in Hz
        1   # 1 = Start, 0 = Stop
    )

def start_telemetry():
    port = find_ranger_port()
    if not port:
        print("Error: Could not find the Ranger Micro.")
        return

    print(f"Connecting to {port} at 57600 baud...")
    connection = mavutil.mavlink_connection(port, baud=57600)

    print("Waiting for Rocket Heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat received! Link is ACTIVE.")

    # MANDATORY STEP: Tell ArduPilot to start sending data
    request_data_streams(connection)

    data = {"alt": 0, "lat": 0, "lon": 0, "pitch": 0, "roll": 0, "v_speed": 0, "batt": 0}
    last_print = time.time()
    
    # Debug counter to see if we are receiving any packets at all
    packet_count = 0

    try:
        while True:
            msg = connection.recv_match(blocking=False)
            if msg:
                packet_count += 1
                msg_type = msg.get_type()
                
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

            # Print every 1 second
            if time.time() - last_print >= 1.0:
                if packet_count < 5:
                    print("Status: Connected, but receiving very few packets. Re-requesting streams...")
                    request_data_streams(connection)
                
                print(f"\n--- ROCKET DATA ---")
                print(f"Battery: {data['batt']:.2f}V")
                print(f"Altitude: {data['alt']:.1f}m")
                print(f"Pitch: {data['pitch']:.1f}° | Roll: {data['roll']:.1f}°")
                print(f"GPS: {data['lat']:.5f}, {data['lon']:.5f}")
                print(f"Packets Rx: {packet_count}")
                
                packet_count = 0
                last_print = time.time()
            
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
        connection.close()

if __name__ == "__main__":
    start_telemetry()