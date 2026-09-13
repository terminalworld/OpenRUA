#!/usr/bin/env python3
"""Grab color+depth+intrinsics+TF for a camera; save npz + png.
Usage: grab.py <cam> [out_prefix]
Then: px.py <prefix> u v  -> world xyz (offline)
"""
import sys, json
import numpy as np, rclpy, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else f"snaps/{cam}"
    rclpy.init()
    node = rclpy.create_node("grab")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    br = CvBridge()
    color = grab(node, f"/{cam}/color/image_raw", Image)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    img = br.imgmsg_to_cv2(color, "bgr8")
    dep = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    Tm = np.eye(4); Tm[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
    Tm[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    cv2.imwrite(out + ".png", img)
    np.savez(out + ".npz", img=img, depth=dep, K=np.array(info.k).reshape(3, 3), T=Tm)
    print(out + ".png/.npz", "depth range", np.nanmin(dep), np.nanmax(dep))
    print("T world<-cam:\n", np.round(Tm, 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
