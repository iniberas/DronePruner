import enum
import math
import os
import pathlib
import sys
import threading
import time
import types

import cv2
import message_filters
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Joy

from mavros_link import MavrosLink
from ibvs_controller import IBVSController
from image_utils import image_to_depth, image_to_rgb, sample_depth
from teleop import JoyTeleop


class Mode(enum.Enum):
    STANDBY = 0   # tidak kirim command apa pun
    MANUAL = 1    # stick gamepad -> velocity
    IBVS = 2      # output IBVS -> velocity
    HOLD = 3      # kirim 0 terus (hover)


def _patch_pathlib_local():
    if "pathlib._local" in sys.modules or hasattr(pathlib, "_local"):
        return
    alias = types.ModuleType("pathlib._local")
    for name in ("Path", "PurePath", "PosixPath", "PurePosixPath",
                 "WindowsPath", "PureWindowsPath"):
        setattr(alias, name, getattr(pathlib, name))
    sys.modules["pathlib._local"] = alias
    pathlib._local = alias


def load_yolo_model(weights_path, repo_dir="/opt/yolov5"):
    _patch_pathlib_local()

    if not os.path.isfile(weights_path):
        raise FileNotFoundError(
            f"weights tidak ditemukan: {os.path.abspath(weights_path)} "
            "(set parameter weights_path:=/path/ke/sengon_s.pt)")

    if repo_dir and os.path.isfile(os.path.join(repo_dir, "hubconf.py")):
        return torch.hub.load(repo_dir, "custom", path=weights_path,
                              source="local", device="cpu")

    print(f"[visual_servo] WARNING: repo YOLOv5 lokal tidak ada di '{repo_dir}', "
          "mencoba download dari GitHub (butuh internet) ...")
    return torch.hub.load("ultralytics/yolov5", "custom",
                          path=weights_path, source="github", device="cpu")


def detect_target(model, rgb_image, target_class_id):
    results = model(rgb_image)
    detections = results.xyxy[0].cpu().numpy()

    best, best_conf = None, -1.0
    for x1, y1, x2, y2, conf, cls in detections:
        if int(cls) == target_class_id and conf > best_conf:
            best, best_conf = (x1, y1, x2, y2), conf

    if best is None:
        return None
    x1, y1, x2, y2 = best
    return ((x1 + x2) / 2.0, (y1 + y2) / 2.0), (int(x1), int(y1), int(x2), int(y2))


class VisualServoNode(Node):
    def __init__(self):
        super().__init__("visual_servo")

        self.declare_parameter("weights_path", "weights/sengon_s.pt")
        self.declare_parameter("yolov5_repo", "/opt/yolov5")
        self.declare_parameter("target_class_id", 0) 
        self.declare_parameter("xt_ratio", 0.5)
        self.declare_parameter("yt_ratio", 0.5)
        self.declare_parameter("zt_meters", 0.5)
        self.declare_parameter("gamma", 0.5)
        self.declare_parameter("detect_timeout", 0.5)
        self.declare_parameter("on_target_lost", "hold")

        self.declare_parameter("input_mode", "joy") # joy/fcu
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("joy_timeout", 0.5)
        self.declare_parameter("stick_override_thresh", 0.5)
        self.declare_parameter("takeoff_alt", 10.0)

        self.declare_parameter("manual_vxy", 1.5)
        self.declare_parameter("manual_vz", 1.0)
        self.declare_parameter("manual_yaw_rate", 0.8)
        self.declare_parameter("accel_lin", 2.0)    # m/s^2
        self.declare_parameter("accel_yaw", 2.0)    # rad/s^2

        self.declare_parameter("cmd_hz", 30.0)
        self.declare_parameter("show_window", True)
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("info_topic", "/camera/camera_info")

        p = self.get_parameter
        self.target_class_id = p("target_class_id").value
        self.xt_ratio = p("xt_ratio").value
        self.yt_ratio = p("yt_ratio").value
        self.zt = p("zt_meters").value
        self.gamma = p("gamma").value
        self.det_timeout = p("detect_timeout").value
        self.on_lost = p("on_target_lost").value
        self.input_mode = p("input_mode").value
        self.joy_timeout = p("joy_timeout").value
        self.override_thresh = p("stick_override_thresh").value
        self.takeoff_alt = p("takeoff_alt").value
        self.manual_scale = np.array([p("manual_vxy").value, p("manual_vxy").value,
                                      p("manual_vz").value, p("manual_yaw_rate").value])
        self.accel = np.array([p("accel_lin").value] * 3 + [p("accel_yaw").value])
        self.show_window = p("show_window").value

        if self.input_mode not in ("joy", "fcu"):
            raise ValueError(f"input_mode tidak dikenal: {self.input_mode}")

        self.ibvs = None
        self.xt = self.yt = None
        self.mode = Mode.STANDBY
        self._cmd = np.zeros(4)
        self._last_cmd_t = time.time()
        self._stick_ready = False
        self._seq = None          # None | "arming" | "climb"
        self._seq_t0 = 0.0
        self._last_arm_req = 0.0
        self._last_mode_req = 0.0

        # shared dengan thread YOLO
        self._lock = threading.Lock()
        self._frame = None
        self._det = None
        self._debug = None
        self._frame_evt = threading.Event()
        self._stop = threading.Event()

        self.get_logger().info("menunggu camera_info ...")
        info_topic = p("info_topic").value
        info = None
        while rclpy.ok() and info is None:
            info = self._wait_for_info(info_topic)
        self._setup_controller(info)

        self.get_logger().info("loading YOLO model ...")
        self.yolo = load_yolo_model(p("weights_path").value, p("yolov5_repo").value)

        self.link = MavrosLink(self)

        if self.input_mode == "joy":
            self.teleop = JoyTeleop(self)
            self.create_subscription(Joy, p("joy_topic").value, self.teleop.on_joy, 10)
            self.mode = Mode.HOLD
        else:
            self.teleop = None

        rgb_sub = message_filters.Subscriber(
            self, Image, p("image_topic").value, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, p("depth_topic").value, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_frames)

        self.debug_pub = self.create_publisher(Image, "/ibvs/debug_image", 1)

        self._worker = threading.Thread(target=self._yolo_worker, daemon=True)
        self._worker.start()

        self.cmd_timer = self.create_timer(1.0 / p("cmd_hz").value, self._cmd_loop)
        self.disp_timer = self.create_timer(1.0 / 15.0, self._display_loop)

        if self.input_mode == "joy":
            self.get_logger().info(
                "mode input: joy | Triangle(tahan)=arm+takeoff, Cross=toggle IBVS, "
                "Circle=HOLD, Square=land, stick (saat IBVS/HOLD)=balik MANUAL")
        else:
            self.get_logger().info("mode input: fcu | IBVS aktif selama mode FCU == GUIDED")

    def _wait_for_info(self, topic):
        box = {}
        sub = self.create_subscription(
            CameraInfo, topic, lambda m: box.setdefault("msg", m), qos_profile_sensor_data)
        t0 = time.time()
        while rclpy.ok() and "msg" not in box and time.time() - t0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return box.get("msg")

    def _setup_controller(self, info):
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        self.width, self.height = info.width, info.height
        self.ibvs = IBVSController(fx, fy, cx, cy, gamma=self.gamma)
        self.xt, self.yt = self.ibvs.pixel_to_normalized(
            self.xt_ratio * self.width, self.yt_ratio * self.height)
        self.get_logger().info(
            f"intrinsik: fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} ({self.width}x{self.height})")

    def _on_frames(self, rgb_msg, depth_msg):
        try:
            frame = (image_to_rgb(rgb_msg), image_to_depth(depth_msg))
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return
        with self._lock:
            self._frame = frame
        self._frame_evt.set()

    def _yolo_worker(self):
        while not self._stop.is_set():
            if not self._frame_evt.wait(0.2):
                continue
            self._frame_evt.clear()
            with self._lock:
                frame, self._frame = self._frame, None
            if frame is None:
                continue
            try:
                self._process_frame(*frame)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"worker error: {e}", throttle_duration_sec=2.0)

    def _process_frame(self, rgb, depth):
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        det = None

        if self.mode == Mode.IBVS:
            result = detect_target(self.yolo, rgb, self.target_class_id)
            if result is not None:
                (xs, ys), (x1, y1, x2, y2) = result
                ix = int(np.clip(xs, 0, self.width - 1))
                iy = int(np.clip(ys, 0, self.height - 1))
                z = sample_depth(depth, ix, iy)

                cv2.rectangle(bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.circle(bgr, (int(xs), int(ys)), 5, (0, 0, 255), -1)
                cv2.putText(bgr, f"Z: {z:.2f}m", (x1, max(y1 - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if math.isfinite(z) and z > 0:
                    det = {"xs": xs, "ys": ys, "z": z, "t": time.time()}

        with self._lock:
            self._det = det
            self._debug = bgr

    def _get_det(self):
        with self._lock:
            return self._det

    def _set_mode(self, new, reason=""):
        if new == self.mode:
            return
        self.get_logger().info(f"mode {self.mode.name} -> {new.name} {reason}".strip())
        self.mode = new
        if new in (Mode.HOLD, Mode.STANDBY):
            self._cmd[:] = 0.0
        if new == Mode.HOLD:
            self._stick_ready = False
        if new != Mode.IBVS:
            with self._lock:
                self._det = None

    def _start_takeoff(self, now):
        if self._seq is not None:
            return
        self.get_logger().info("takeoff: GUIDED -> arm -> takeoff")
        self.link.request_mode("GUIDED")
        self._last_mode_req = now
        self._seq, self._seq_t0, self._last_arm_req = "arming", now, 0.0

    def _start_land(self):
        self._seq = None
        self.link.request_land()
        self._set_mode(Mode.MANUAL, "(landing)")

    def _tick_sequence(self, now):
        if self._seq == "arming":
            if now - self._seq_t0 > 90.0:
                self.get_logger().error("arm timeout")
                self._seq = None
            elif self.link.armed:
                if self.link.z > 0.5:
                    self._seq = None
                    self._set_mode(Mode.MANUAL, "(sudah terbang)")
                else:
                    self.link.request_takeoff(self.takeoff_alt)
                    self._seq, self._seq_t0 = "climb", now
            else:
                if self.link.mode != "GUIDED" and now - self._last_mode_req > 2.0:
                    self.link.request_mode("GUIDED")
                    self._last_mode_req = now
                if now - self._last_arm_req > 2.0:
                    self.link.request_arm()
                    self._last_arm_req = now
        elif self._seq == "climb":
            if self.link.z >= self.takeoff_alt - 0.3:
                self.get_logger().info(f"altitude tercapai: {self.link.z:.2f} m")
                self._seq = None
                self._set_mode(Mode.MANUAL, "(takeoff selesai)")
            elif now - self._seq_t0 > 30.0:
                self.get_logger().warn("timeout menunggu altitude takeoff")
                self._seq = None
                self._set_mode(Mode.MANUAL)

    def _update_mode_joy(self, now):
        self._tick_sequence(now)
        events = self.teleop.poll(now)
        mag = self.teleop.magnitude()

        # failsafe
        if self.teleop.age(now) > self.joy_timeout:
            if self.mode != Mode.HOLD:
                self.get_logger().warn("/joy putus -> HOLD")
            self._set_mode(Mode.HOLD)
            return

        # IBVS cuma boleh klo masih GUIDED
        if self._seq is None and self.link.mode != "GUIDED" and self.mode == Mode.IBVS:
            self._set_mode(Mode.MANUAL, f"(FCU mode {self.link.mode}, bukan GUIDED)")

        if self.mode == Mode.IBVS and mag > self.override_thresh:
            self._set_mode(Mode.MANUAL, "(stick override)")
        elif self.mode == Mode.HOLD:
            if mag < 0.1:
                self._stick_ready = True
            elif self._stick_ready and mag > self.override_thresh:
                self._set_mode(Mode.MANUAL, "(stick)")

        for ev in events:
            if ev == "hold":
                self._set_mode(Mode.HOLD, "(tombol)")
            elif ev == "land":
                self._start_land()
            elif ev == "takeoff":
                self._start_takeoff(now)
            elif ev == "ibvs":
                if self.mode == Mode.IBVS:
                    self._set_mode(Mode.MANUAL, "(toggle)")
                elif self.link.mode == "GUIDED" and self._seq is None:
                    self._set_mode(Mode.IBVS, "(toggle)")
                else:
                    self.get_logger().warn("IBVS ditolak: FCU belum GUIDED / lagi takeoff")

    def _update_mode_fcu(self):
        self._set_mode(Mode.IBVS if self.link.mode == "GUIDED" else Mode.STANDBY)

    def _target_command(self):
        if self.mode == Mode.MANUAL:
            return np.array(self.teleop.sticks) * self.manual_scale if self.teleop else np.zeros(4)

        if self.mode == Mode.IBVS:
            det = self._get_det()
            if det is None or time.time() - det["t"] > self.det_timeout:
                if self.on_lost == "manual" and self.teleop is not None:
                    self._set_mode(Mode.MANUAL, "(target hilang)")
                return np.zeros(4)
            x, y = self.ibvs.pixel_to_normalized(det["xs"], det["ys"])
            vc_cam = self.ibvs.compute_velocity(x, y, det["z"], self.xt, self.yt, self.zt)
            return np.asarray(self.ibvs.camera_to_body_velocity(vc_cam), dtype=float)

        return np.zeros(4)  # HOLD / STANDBY

    def _cmd_loop(self):
        now = time.time()
        dt = min(max(now - self._last_cmd_t, 1e-3), 0.2)
        self._last_cmd_t = now

        if self.input_mode == "joy":
            self._update_mode_joy(now)
        else:
            self._update_mode_fcu()

        guided = self.link.mode == "GUIDED"
        if self.mode == Mode.STANDBY or not guided or self._seq is not None:
            self._cmd[:] = 0.0
            return

        tgt = self._target_command()
        if self.mode == Mode.HOLD:
            self._cmd[:] = 0.0
        else:
            lim = self.accel * dt
            self._cmd = self._cmd + np.clip(tgt - self._cmd, -lim, lim)
        self.link.send_body_velocity(*self._cmd)

    def _display_loop(self):
        with self._lock:
            bgr = self._debug
        if bgr is None:
            return
        img = bgr.copy()
        label = self.mode.name + (f" [{self._seq}]" if self._seq else "")
        cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.debug_pub.publish(msg)

        if self.show_window:
            cv2.imshow("YOLO Target Tracker", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("'q' ditekan, berhenti")
                rclpy.shutdown()

    def stop(self):
        self._stop.set()
        try:
            self._worker.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.link.send_body_velocity(0, 0, 0, 0)
        except Exception:
            pass


def main():
    rclpy.init()
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()