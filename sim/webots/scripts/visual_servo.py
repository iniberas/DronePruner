import math
import time
import socket
import struct
import numpy as np
import torch
import cv2

from ibvs_controller import IBVSController
from ardupilot_link import ArduPilotLink

YOLO_WEIGHTS_PATH = "weights/sengon_s.pt"
TARGET_CLASS_ID = 0

XT_RATIO = 0.5
YT_RATIO = 0.5
ZT_METERS = 0.5
GAMMA = 0.5

MAVLINK_CONN = "udp:127.0.0.1:14550"
TAKEOFF_ALT = 2.0
LOOP_HZ = 15

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FOV = 1.0

MAX_RANGE = 15.0

def load_yolo_model(weights_path):
    model = torch.hub.load("ultralytics/yolov5", "custom",
                            path=weights_path, source="github", device="cpu")
    model.eval()
    return model

def detect_target(model, rgb_image, target_class_id):
    results = model(rgb_image)
    detections = results.xyxy[0].cpu().numpy()

    best = None
    best_conf = -1.0
    for x1, y1, x2, y2, conf, cls in detections:
        if int(cls) == target_class_id and conf > best_conf:
            best = (x1, y1, x2, y2)
            best_conf = conf

    if best is None:
        return None

    x1, y1, x2, y2 = best
    xs = (x1 + x2) / 2.0
    ys = (y1 + y2) / 2.0
    return (xs, ys), (int(x1), int(y1), int(x2), int(y2))

def get_camera_intrinsics(width, height, fov):
    fx = width / (2.0 * math.tan(fov / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return fx, fy, cx, cy

def receive_image_from_socket(sock, is_depth=False):
    header_size = struct.calcsize("=HH")
    header = b""
    while len(header) < header_size:
        chunk = sock.recv(header_size - len(header))
        if not chunk:
            return None
        header += chunk

    width, height = struct.unpack("=HH", header)

    bytes_to_read = width * height
    img_bytes = bytearray()
    while len(img_bytes) < bytes_to_read:
        chunk = sock.recv(min(bytes_to_read - len(img_bytes), 4096))
        if not chunk:
            return None
        img_bytes += chunk

    img_raw = np.frombuffer(img_bytes, dtype=np.uint8).reshape((height, width))

    if is_depth:
        return img_raw
    else:
        img_rgb = cv2.cvtColor(img_raw, cv2.COLOR_GRAY2RGB)
        return img_rgb

def main():
    fx, fy, cx, cy = get_camera_intrinsics(CAM_WIDTH, CAM_HEIGHT, CAM_FOV)
    xt_px = XT_RATIO * CAM_WIDTH
    yt_px = YT_RATIO * CAM_HEIGHT

    ibvs = IBVSController(fx, fy, cx, cy, gamma=GAMMA)
    xt, yt = ibvs.pixel_to_normalized(xt_px, yt_px)

    print("[main_pruner] loading YOLO model...")
    yolo_model = load_yolo_model(YOLO_WEIGHTS_PATH)

    print("[main_pruner] Menyambungkan ke Webots stream...")
    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cam_sock.connect(('127.0.0.1', 5555))
    
    depth_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    depth_sock.connect(('127.0.0.1', 5556))

    link = ArduPilotLink(MAVLINK_CONN)
    link.wait_until_armable_and_guided(altitude=TAKEOFF_ALT)

    period = 1.0 / LOOP_HZ
    print("[main_pruner] Mulai loop visual servoing...")

    while True:
        t0 = time.time()

        rgb = receive_image_from_socket(cam_sock, is_depth=False)
        depth_map = receive_image_from_socket(depth_sock, is_depth=True)
        
        if rgb is None or depth_map is None:
            continue

        detection_result = detect_target(yolo_model, rgb, TARGET_CLASS_ID)
        bgr_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if detection_result is None:
            link.send_body_velocity(0, 0, 0, 0)
        else:
            (xs, ys), (x1, y1, x2, y2) = detection_result
            
            ix = int(np.clip(xs, 0, CAM_WIDTH - 1))
            iy = int(np.clip(ys, 0, CAM_HEIGHT - 1))
            z  = (depth_map[iy, ix] / 255.0) * MAX_RANGE

            cv2.rectangle(bgr_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(bgr_image, (int(xs), int(ys)), 5, (0, 0, 255), -1)
            cv2.putText(bgr_image, f"Z: {z:.2f}m", (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if z <= 0 or math.isinf(z) or math.isnan(z):
                link.send_body_velocity(0, 0, 0, 0)
            else:
                x, y = ibvs.pixel_to_normalized(xs, ys)
                vc_cam = ibvs.compute_velocity(x, y, z, xt, yt, ZT_METERS)
                vx, vy, vz, omega = ibvs.camera_to_body_velocity(vc_cam)
                link.send_body_velocity(vx, vy, vz, omega)

        cv2.imshow("YOLO Target Tracker", bgr_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)

if __name__ == "__main__":
    main()