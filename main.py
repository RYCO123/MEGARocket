import time
import sys
from pymavlink import mavutil
from pynput import keyboard

class RocketCommander:
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        # source_system=255 identifies this script as a 'Pilot/GCS' 
        # This is required for ArduPilot to accept WASD steering (RC Overrides)
        print(f"Connecting to Rocket via MAVProxy on {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string, source_system=255)
        
        # Internal State Variables
        self.altitude = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.mode = "UNKNOWN"
        self.armed = False
        
        # Steering values (1500 is neutral)
        # Mapping: Ch1: Roll, Ch2: Pitch, Ch4: Yaw
        self.controls = {1: 1500, 2: 1500, 4: 1500}

    def wait_for_heartbeat(self):
        print("Waiting for Rocket Heartbeat...")
        self.master.wait_heartbeat()
        print(f"Connected! Target System: {self.master.target_system}")

    def set_mode(self, mode_name):
        """Changes flight mode (MANUAL, QSTABILIZE, FBWA)."""
        if mode_name not in self.master.mode_mapping():
            print(f"Error: Mode {mode_name} not available.")
            return
        
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"\n[CMD] Mode -> {mode_name}")

    def arm_disarm(self, arm_state):
        """True to Arm, False to Disarm. Uses bypass 21196."""
        val = 1 if arm_state else 0
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, val, 21196, 0, 0, 0, 0, 0
        )
        print(f"\n[CMD] {'ARMING' if arm_state else 'DISARMING'}")

    def send_rc_overrides(self):
        """Sends digital stick positions. MUST BE ARMED to work."""
        # 65535 tells ArduPilot to 'ignore' that channel (used for Throttle/Ch3)
        self.master.mav.rc_channels_override_send(
            self.master.target_system, 
            self.master.target_component,
            self.controls[1], # Ch 1: Roll
            self.controls[2], # Ch 2: Pitch
            65535,            # Ch 3: Throttle (Ignored)
            self.controls[4], # Ch 4: Yaw
            0, 0, 0, 0
        )

    def update_telemetry(self):
        """Reads data. Filters for System 1 to avoid MAVProxy confusion."""
        msg = self.master.recv_match(blocking=False)
        if not msg:
            return

        msg_type = msg.get_type()
        src_sys = msg.get_srcSystem()

        # ONLY listen to the Rocket (System 1), ignore MAVProxy (System 255)
        if src_sys == 1:
            if msg_type == 'HEARTBEAT':
                # Check if Rocket is Armed
                self.armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                # Resolve Mode Name
                for name, id in self.master.mode_mapping().items():
                    if id == msg.custom_mode:
                        self.mode = name

            elif msg_type == 'ATTITUDE':
                self.pitch = msg.pitch * 57.2958 # Rad to Deg
                self.roll = msg.roll * 57.2958

            elif msg_type == 'VFR_HUD':
                self.altitude = msg.alt

    def on_press(self, key):
        try:
            # Mode Control
            if key.char == '1': self.set_mode('MANUAL')
            if key.char == '2': self.set_mode('QSTABILIZE')
            if key.char == '3': self.set_mode('FBWA')
            if key.char == 'z': self.arm_disarm(True)
            
            # WASD Steering
            if key.char == 'w': self.controls[2] = 1100 # Pitch
            if key.char == 's': self.controls[2] = 1900 
            if key.char == 'a': self.controls[1] = 1100 # Roll
            if key.char == 'd': self.controls[1] = 1900
            if key.char == 'q': self.controls[4] = 1100 # Yaw
            if key.char == 'e': self.controls[4] = 1900

        except AttributeError:
            if key == keyboard.Key.space: self.arm_disarm(False)

    def on_release(self, key):
        # Reset to Neutral (1500) when key is released
        self.controls = {1: 1500, 2: 1500, 4: 1500}

    def run(self):
        self.wait_for_heartbeat()
        
        # Keyboard Listener
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        print("\n--- MISSION CONTROL ACTIVE ---")
        print("1:Manual | 2:Rocket | 3:Glider | Z:Arm | Space:Disarm | W,A,S,D:Steer")
        
        try:
            while True:
                self.update_telemetry()
                
                # Digital Heartbeat: Required for Overrides to be accepted
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, 
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

                # Send Steering if Armed
                if self.armed:
                    self.send_rc_overrides()

                # HUD Display
                arm_status = "ARMED" if self.armed else "DISARMED"
                print(f"[{arm_status}] Mode: {self.mode:11} | Alt: {self.altitude:5.1f}m | Pitch: {self.pitch:5.1f}°", end='\r')
                
                time.sleep(0.05) # 20Hz update
        except KeyboardInterrupt:
            print("\nShutting down.")
            sys.exit(0)

if __name__ == "__main__":
    commander = RocketCommander()
    commander.run()