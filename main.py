import os
import select
import time
import sys
import termios
import _thread
import threading
import tty
from pymavlink import mavutil
from pynput import keyboard


class RocketCommander:
    def __init__(self, connection_string='udp:127.0.0.1:14552'):
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
        self.mode_slots = {}
        self.pressed_keys = set()
        self.parachute_released = False
        self._status_line = ""
        self._stdin_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._stdin_attrs = None
        self._shutdown = False
        
        # Standard fixed-wing RC axis mapping.
        # Ch1 roll -> elevons, Ch2 pitch -> elevons, Ch4 yaw -> rudders.
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
        self.mode = mode_name
        self.log_event(f"[CMD] Mode -> {mode_name}")

    def fetch_param_value(self, param_name, timeout=1.5, attempts=3):
        for _ in range(attempts):
            self.master.param_fetch_one(param_name)
            deadline = time.time() + timeout
            while time.time() < deadline:
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
                if msg is None:
                    continue
                if isinstance(msg.param_id, bytes):
                    pid = msg.param_id.decode(errors='ignore').rstrip('\x00')
                else:
                    pid = str(msg.param_id).rstrip('\x00')
                if pid == param_name:
                    return float(msg.param_value)
        return None

    def load_mode_slots(self):
        """Load FLTMODE1..6 assignments so number keys match QGC/ArduPlane mode slots."""
        inverse_mode_map = {mode_id: name for name, mode_id in self.master.mode_mapping().items()}
        slots = {}
        for slot in range(1, 7):
            param_name = f"FLTMODE{slot}"
            value = self.fetch_param_value(param_name)
            if value is None:
                continue
            mode_id = int(round(value))
            slots[str(slot)] = inverse_mode_map.get(mode_id, f"MODE_{mode_id}")
        self.mode_slots = slots
        if self.mode_slots:
            label = " | ".join(f"{k}:{v}" for k, v in sorted(self.mode_slots.items()))
            print(f"Loaded mode slots -> {label}")
        else:
            print("Warning: Could not read FLTMODE1..6. Number keys fall back to 1:MANUAL 2:QSTABILIZE 3:FBWA.")

    def set_mode_from_slot(self, slot_key):
        mode_name = self.mode_slots.get(slot_key)
        if mode_name:
            self.set_mode(mode_name)
            return
        fallback = {'1': 'MANUAL', '2': 'QSTABILIZE', '3': 'FBWA'}
        if slot_key in fallback:
            self.set_mode(fallback[slot_key])
        else:
            print(f"\n[CMD] Slot {slot_key} is not configured.")

    def arm_disarm(self, arm_state):
        """True to Arm, False to Disarm. Uses bypass 21196."""
        val = 1 if arm_state else 0
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, val, 21196, 0, 0, 0, 0, 0
        )
        # Reflect operator intent immediately in the HUD; heartbeat will confirm.
        self.armed = arm_state
        self.log_event(f"[CMD] {'ARMING' if arm_state else 'DISARMING'}")

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

    def release_parachute(self):
        """One-shot parachute release through ArduPilot's parachute subsystem."""
        if self.parachute_released:
            self.log_event("[CMD] Parachute already released")
            return

        ok, ack_result = self.send_parachute_command(mavutil.mavlink.PARACHUTE_RELEASE)
        status = "ACK" if ok else f"ack={ack_result}" if ack_result is not None else "sent/no-ack"
        self.log_event(f"[CMD] Parachute RELEASE [{status}]")
        if ok:
            self.parachute_released = True

    def send_parachute_command(self, action, attempts=2):
        for _ in range(attempts):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_PARACHUTE,
                0,
                action,
                0, 0, 0, 0, 0
            )
            deadline = time.time() + 0.6
            while time.time() < deadline:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_PARACHUTE:
                    # ACCEPTED(0) and IN_PROGRESS(1) are both useful positives here.
                    return msg.result in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    ), msg.result
                time.sleep(0.02)
        return False, None

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

    def key_token(self, key):
        try:
            if key.char is not None:
                return key.char.lower()
        except AttributeError:
            pass
        return key

    def log_event(self, text):
        # Move event output to its own line and preserve the single-line HUD.
        print(f"\n{text}")
        if self._status_line:
            print(f"\r\033[K{self._status_line}", end="", flush=True)

    def handle_discrete_command(self, ch):
        if ch in ('1', '2', '3', '4', '5', '6'):
            self.set_mode_from_slot(ch)
        elif ch == 'z':
            self.arm_disarm(True)
        elif ch == 'f':
            self.release_parachute()
        elif ch == ' ':
            self.arm_disarm(False)

    def on_press(self, key):
        token = self.key_token(key)
        if token in self.pressed_keys:
            return
        self.pressed_keys.add(token)

        try:
            ch = key.char.lower()
            # WASD Steering
            if ch == 'w':
                self.controls[2] = 1100  # Pitch
            if ch == 's':
                self.controls[2] = 1900
            if ch == 'a':
                self.controls[1] = 1100  # Roll
            if ch == 'd':
                self.controls[1] = 1900
            if ch == 'q':
                self.controls[4] = 1100  # Yaw
            if ch == 'e':
                self.controls[4] = 1900
        except AttributeError:
            pass

    def on_release(self, key):
        token = self.key_token(key)
        self.pressed_keys.discard(token)
        if token in ('w', 's'):
            self.controls[2] = 1500
        if token in ('a', 'd'):
            self.controls[1] = 1500
        if token in ('q', 'e'):
            self.controls[4] = 1500

    def run(self):
        self.wait_for_heartbeat()
        self.load_mode_slots()
        
        # Keyboard Listener
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        tty_thread = None
        if self._stdin_fd is not None:
            tty_thread = threading.Thread(target=self.tty_key_listener, daemon=True)
            tty_thread.start()
        else:
            cmd_thread = threading.Thread(target=self.command_listener, daemon=True)
            cmd_thread.start()

        print("\n--- MISSION CONTROL ACTIVE ---")
        print("1-6:QGC Flight Modes | Z:Arm | Space:Disarm | W/S:Pitch | A/D:Roll | Q/E:Yaw | F:Parachute Release")
        print("Terminal commands: arm | disarm")
        print("")
        
        try:
            while True:
                self.update_telemetry()
                
                # Digital Heartbeat: Required for Overrides to be accepted
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, 
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

                # Always send steering overrides while script is active.
                self.send_rc_overrides()

                # HUD Display
                arm_status = "ARMED" if self.armed else "DISARMED"
                self._status_line = f"[{arm_status}] Mode: {self.mode:11}"
                print(f"\r\033[K{self._status_line}", end='', flush=True)
                
                time.sleep(0.05) # 20Hz update
        except KeyboardInterrupt:
            print("\nShutting down.")
            sys.exit(0)
        finally:
            self._shutdown = True
            self.restore_tty()

    def tty_key_listener(self):
        if self._stdin_fd is None:
            return

        self._stdin_attrs = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        while not self._shutdown:
            ready, _, _ = select.select([self._stdin_fd], [], [], 0.1)
            if not ready:
                continue
            ch = os.read(self._stdin_fd, 1).decode(errors='ignore')
            if not ch:
                continue
            if ch == '\x03':
                _thread.interrupt_main()
                return
            if ch in ('1', '2', '3', '4', '5', '6', 'z', 'f', ' '):
                self.handle_discrete_command(ch)

    def restore_tty(self):
        if self._stdin_fd is not None and self._stdin_attrs is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_attrs)
            self._stdin_attrs = None

    def command_listener(self):
        while True:
            try:
                cmd = input().strip().lower()
            except EOFError:
                return
            if cmd == 'arm':
                self.arm_disarm(True)
            elif cmd == 'disarm':
                self.arm_disarm(False)

if __name__ == "__main__":
    commander = RocketCommander()
    commander.run()
