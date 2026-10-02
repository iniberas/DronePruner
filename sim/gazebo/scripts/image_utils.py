import numpy as np


def image_to_rgb(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"encoding is not supported: {msg.encoding}")
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def image_to_depth(msg):
    if msg.encoding != "32FC1":
        raise ValueError(f"encoding is not supported: {msg.encoding}")
    buf = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
    return np.ascontiguousarray(buf[:, : msg.width])


def sample_depth(depth, ix, iy, r=2):
    patch = depth[max(iy - r, 0): iy + r + 1, max(ix - r, 0): ix + r + 1]
    valid = patch[np.isfinite(patch) & (patch > 0)]
    return float(np.median(valid)) if valid.size else float("nan")
