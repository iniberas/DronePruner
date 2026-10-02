import math
import os
import pathlib
import sys
import time
import types

import cv2
import message_filters
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from ardupilot_link import ArduPilotLink
from ibvs_controller import IBVSController
from image_utils import image_to_depth, image_to_rgb, sample_depth


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
        self.declare_parameter("mavlink_conn", "udp:127.0.0.1:14550")
        self.declare_parameter("takeoff_alt", 2.0)
        self.declare_parameter("loop_hz", 15.0)
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
        self.show_window = p("show_window").value

        self.ibvs = None
        self.xt = self.yt = None
        self.latest = None
        self.last_frame_time = time.time()

        self.get_logger().info("menunggu camera_info ...")
        info_topic = p("info_topic").value
        info = None
        while rclpy.ok() and info is None:
            info = self._wait_for_info(info_topic)
        self._setup_controller(info)

        self.get_logger().info("loading YOLO model ...")
        self.yolo = load_yolo_model(p("weights_path").value, p("yolov5_repo").value)

        self.link = ArduPilotLink(p("mavlink_conn").value)
        self.link.wait_until_armable_and_guided(altitude=p("takeoff_alt").value)

        rgb_sub = message_filters.Subscriber(
            self, Image, p("image_topic").value, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(
            self, Image, p("depth_topic").value, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._on_frames)

        self.debug_pub = self.create_publisher(Image, "/ibvs/debug_image", 1)
        self.timer = self.create_timer(1.0 / p("loop_hz").value, self._control_loop)
        self.get_logger().info("mulai loop visual servoing")

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
            self.latest = (image_to_rgb(rgb_msg), image_to_depth(depth_msg))
            self.last_frame_time = time.time()
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)

    def _control_loop(self):
        if self.latest is None:
            if time.time() - self.last_frame_time > 1.0:
                self.link.send_body_velocity(0, 0, 0, 0)
            return
        rgb, depth = self.latest
        self.latest = None

        result = detect_target(self.yolo, rgb, self.target_class_id)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if result is None:
            self.link.send_body_velocity(0, 0, 0, 0)
        else:
            (xs, ys), (x1, y1, x2, y2) = result
            ix = int(np.clip(xs, 0, self.width - 1))
            iy = int(np.clip(ys, 0, self.height - 1))
            z = sample_depth(depth, ix, iy)

            cv2.rectangle(bgr, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(bgr, (int(xs), int(ys)), 5, (0, 0, 255), -1)
            cv2.putText(bgr, f"Z: {z:.2f}m", (x1, max(y1 - 10, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if not math.isfinite(z) or z <= 0:
                self.link.send_body_velocity(0, 0, 0, 0)
            else:
                x, y = self.ibvs.pixel_to_normalized(xs, ys)
                vc_cam = self.ibvs.compute_velocity(x, y, z, self.xt, self.yt, self.zt)
                vx, vy, vz, omega = self.ibvs.camera_to_body_velocity(vc_cam)
                self.link.send_body_velocity(vx, vy, vz, omega)

        self._publish_debug(bgr)
        if self.show_window:
            cv2.imshow("YOLO Target Tracker", bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("'q' ditekan, berhenti")
                rclpy.shutdown()

    def _publish_debug(self, bgr):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        self.debug_pub.publish(msg)

    def stop(self):
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