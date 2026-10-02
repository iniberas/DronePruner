import time
from pymavlink import mavutil


class ArduPilotLink:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        print(f"[ardupilot_link] connecting to {connection_string} ...")
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print("[ardupilot_link] heartbeat ok, sysid:",
              self.master.target_system, "compid:", self.master.target_component)
        self._armed = False
        self._set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)

    def _set_message_interval(self, msg_id, hz):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
        )

    def _poll(self, timeout=0.5):
        msg = self.master.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.get_srcSystem() == self.master.target_system \
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
            self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif t == "STATUSTEXT":
            print(f"[ardupilot_link] FCU: {msg.text}")
        return msg

    def set_mode(self, mode_name="GUIDED"):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[ardupilot_link] mode -> {mode_name}")

    def arm(self, wait=True, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            if not wait:
                return True
            deadline = time.time() + 2.0
            while time.time() < deadline:
                self._poll(0.3)
                if self._armed:
                    print("[ardupilot_link] armed")
                    return True
        raise RuntimeError("[ardupilot_link] arm failed (timeout).")

    def takeoff(self, altitude=2.0):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        print(f"[ardupilot_link] takeoff -> {altitude} m")

    def wait_altitude(self, altitude, tol=0.3, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self._poll(0.5)
            if msg is not None and msg.get_type() == "LOCAL_POSITION_NED":
                if -msg.z >= altitude - tol:
                    print(f"[ardupilot_link] altitude reached: {-msg.z:.2f} m")
                    return True
        print("[ardupilot_link] WARNING: timeout waiting for takeoff altitude, proceeding anyway")
        return False

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

    def wait_until_armable_and_guided(self, altitude=2.0, arm_timeout=90):
        self.set_mode("GUIDED")
        time.sleep(1)
        self.arm(timeout=arm_timeout)
        self.takeoff(altitude)
        self.wait_altitude(altitude)
