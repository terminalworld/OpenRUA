#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera once; convert pixels to world."""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def _grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Cam:
    def __init__(self, node, name, tfbuf):
        self.name = name
        bridge = CvBridge()
        self.color = bridge.imgmsg_to_cv2(_grab(node, f"/{name}/color/image_raw", Image), "bgr8")
        self.depth = bridge.imgmsg_to_cv2(_grab(node, f"/{name}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = _grab(node, f"/{name}/color/camera_info", CameraInfo)
        self.fx, self.fy, self.cx, self.cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{name}_optical_frame"
        import time
        end = time.time() + 10
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tfbuf.can_transform("world", frame, rclpy.time.Time()):
                break
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        self.T = np.eye(4)
        self.T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        self.T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

    def px2world(self, u, v):
        z = float(self.depth[int(v), int(u)])
        if not np.isfinite(z) or z <= 0:
            return None
        p = np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z, 1.0])
        return (self.T @ p)[:3]

    def cloud(self, mask):
        vs, us = np.nonzero(mask)
        z = self.depth[vs, us]
        ok = np.isfinite(z) & (z > 0)
        us, vs, z = us[ok], vs[ok], z[ok]
        P = np.stack([(us - self.cx) * z / self.fx, (vs - self.cy) * z / self.fy, z, np.ones_like(z)])
        return (self.T @ P)[:3].T


def grab(names):
    rclpy.init()
    node = rclpy.create_node("camlib")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    cams = {n: Cam(node, n, tfbuf) for n in names}
    node.destroy_node()
    rclpy.shutdown()
    return cams


if __name__ == "__main__":
    name = sys.argv[1]
    cams = grab([name])
    c = cams[name]
    cv2.imwrite(f"{name}.png", c.color)
    np.save(f"{name}_depth.npy", c.depth)
    np.save(f"{name}_T.npy", c.T)
    np.save(f"{name}_K.npy", np.array([c.fx, c.fy, c.cx, c.cy]))
    for a in sys.argv[2:]:
        u, v = map(int, a.split(","))
        print(a, c.px2world(u, v))
