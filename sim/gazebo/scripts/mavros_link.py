import time
import rclpy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.qos import qos_profile_sensor_data


class MavrosLink:
    def __init__(self, node, ns="/mavros"):
        self.node = node
        self.state = State()
        self.z = 0.0

        node.create_subscription(State, f"{ns}/state", self._on_state, qos_profile_sensor_data)
        node.create_subscription(PoseStamped, f"{ns}/local_position/pose",
                                 self._on_pose, qos_profile_sensor_data)
        self.vel_pub = node.create_publisher(
            TwistStamped, f"{ns}/setpoint_velocity/cmd_vel", 10)

        self.cli_mode = node.create_client(SetMode, f"{ns}/set_mode")
        self.cli_arm = node.create_client(CommandBool, f"{ns}/cmd/arming")
        self.cli_takeoff = node.create_client(CommandTOL, f"{ns}/cmd/takeoff")

        print("[mavros_link] menunggu koneksi FCU ...")
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(node, timeout_sec=0.2)
        print("[mavros_link] FCU terhubung")

    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.z = msg.pose.position.z

    @property
    def mode(self):
        return self.state.mode

    @property
    def armed(self):
        return self.state.armed

    @property
    def connected(self):
        return self.state.connected

    def _call(self, client, req, timeout=5.0):
        client.wait_for_service(timeout_sec=timeout)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def set_mode(self, mode="GUIDED"):
        req = SetMode.Request()
        req.custom_mode = mode
        res = self._call(self.cli_mode, req)
        print(f"[mavros_link] mode -> {mode}: {res.mode_sent if res else 'no reply'}")

    def arm(self, timeout=90):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            req = CommandBool.Request()
            req.value = True
            self._call(self.cli_arm, req)
            t1 = time.time()
            while time.time() - t1 < 2.0:
                rclpy.spin_once(self.node, timeout_sec=0.2)
                if self.state.armed:
                    print("[mavros_link] armed")
                    return True
        raise RuntimeError("[mavros_link] arm failed (timeout).")

    def takeoff(self, altitude=2.0):
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        self._call(self.cli_takeoff, req)
        print(f"[mavros_link] takeoff -> {altitude} m")

    def wait_altitude(self, altitude, tol=0.3, timeout=30):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.z >= altitude - tol:
                print(f"[mavros_link] altitude reached: {self.z:.2f} m")
                return True
        print("[mavros_link] WARNING: timeout waiting for takeoff altitude")
        return False

    def wait_until_armable_and_guided(self, altitude=2.0, arm_timeout=90):
        self.set_mode("GUIDED")
        time.sleep(1)
        self.arm(timeout=arm_timeout)
        self.takeoff(altitude)
        self.wait_altitude(altitude)

    def _call_nb(self, client, req, label, ok_fn):
        log = self.node.get_logger()
        if not client.service_is_ready():
            log.warn(f"[mavros_link] service {label} belum siap")
            return

        def _done(fut):
            try:
                res = fut.result()
            except Exception as e:  # noqa: BLE001
                log.error(f"[mavros_link] {label} error: {e}")
                return
            log.info(f"[mavros_link] {label}: {'ok' if ok_fn(res) else 'DITOLAK/gagal'}")

        client.call_async(req).add_done_callback(_done)

    def request_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self._call_nb(self.cli_mode, req, f"set_mode {mode}", lambda r: r.mode_sent)

    def request_arm(self):
        req = CommandBool.Request()
        req.value = True
        self._call_nb(self.cli_arm, req, "arm", lambda r: r.success)

    def request_takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        self._call_nb(self.cli_takeoff, req, f"takeoff {altitude} m", lambda r: r.success)

    def request_land(self):
        self.request_mode("LAND")

    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(-vy)
        msg.twist.linear.z = float(-vz)
        msg.twist.angular.z = float(-yaw_rate)
        self.vel_pub.publish(msg)