#!/usr/bin/env python3
"""Snapshot color (and optionally depth) frames from one or more cameras.

usage: cam_snap.py <cam> [<cam> ...] [--depth]
writes <cam>.png (color) and, with --depth, <cam>_depth.npy (float32 metres).
"""
import sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

args = [a for a in sys.argv[1:] if not a.startswith("--")]
want_depth = "--depth" in sys.argv
rclpy.init()
node = rclpy.create_node("cam_snap")
bridge = CvBridge()
got = {}
subs = []
for cam in args:
    topics = {f"{cam}.png": f"/{cam}/color/image_raw"}
    if want_depth:
        topics[f"{cam}_depth.npy"] = f"/{cam}/depth/image_raw"
    for out, topic in topics.items():
        def cb(msg, out=out):
            got.setdefault(out, msg)
        subs.append(node.create_subscription(Image, topic, cb, qos_profile_sensor_data))
        subs.append(node.create_subscription(Image, topic, cb, 1))
need = len(args) * (2 if want_depth else 1)
import time
t0 = time.time()
while len(got) < need and time.time() - t0 < 60:
    rclpy.spin_once(node, timeout_sec=0.2)
for out, msg in got.items():
    if out.endswith(".png"):
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(out, img)
        print(out, img.shape)
    else:
        d = bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)
        np.save(out, d)
        print(out, d.shape, "min", np.nanmin(d), "max", np.nanmax(d))
missing = need - len(got)
if missing:
    print("MISSING", missing, "frames", file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
