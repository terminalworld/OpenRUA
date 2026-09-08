#!/usr/bin/env python3
"""Save one frame from an image topic to a PNG.

Usage: python3 tools/cam_snap.py <camera_name> [out.png]
Reads /<camera_name>/color/image_raw (pass a full topic to use it as-is).
"""
import sys

import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    cam = sys.argv[1]
    topic = cam if cam.startswith("/") else f"/{cam}/color/image_raw"
    out = sys.argv[2] if len(sys.argv) > 2 else f"{cam.strip('/').split('/')[0]}.png"
    rclpy.init()
    node = rclpy.create_node("cam_snap")
    got: list = []
    node.create_subscription(Image, topic, lambda m: got.append(m), 1)
    while not got:
        rclpy.spin_once(node, timeout_sec=1.0)
    msg = got[0]
    if "FC" in msg.encoding or "16UC" in msg.encoding:  # depth image
        import numpy as np
        depth = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
        np.save(out.rsplit(".", 1)[0] + ".npy", depth)  # raw meters
        finite = depth[np.isfinite(depth)]
        lo, hi = (finite.min(), finite.max()) if finite.size else (0.0, 1.0)
        cv2.imwrite(out, ((depth - lo) / max(hi - lo, 1e-6) * 255).clip(0, 255)
                    .astype("uint8"))
        print(f"{out} (visualization; raw meters in .npy)")
    else:
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        print(out)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
