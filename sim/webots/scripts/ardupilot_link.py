import time
from pymavlink import mavutil


class ArduPilotLink:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        print(f"[ardupilot_link] connecting to {connection_string} ...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("[ardupilot_link] heartbeat ok, sysid:",
              self.master.target_system, "compid:", self.master.target_component)

    def set_mode(self, mode_name="GUIDED"):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[ardupilot_link] mode -> {mode_name}")

    def arm(self, wait=True):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        if wait:
            self.master.motors_armed_wait()
        print("[ardupilot_link] armed")

    def takeoff(self, altitude=2.0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        print(f"[ardupilot_link] takeoff -> {altitude} m")

    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        type_mask = 0b0000011111000111  # ignore pos, accel, yaw
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0,
            yaw_rate,
        )

    def wait_until_armable_and_guided(self, altitude=2.0, arm_timeout=30):
        self.set_mode("GUIDED")
        time.sleep(1)
        self.arm()
        self.takeoff(altitude)
        time.sleep(5)