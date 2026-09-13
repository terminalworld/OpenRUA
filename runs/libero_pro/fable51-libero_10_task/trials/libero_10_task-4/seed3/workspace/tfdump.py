#!/usr/bin/env python3
"""Dump all TF frames and a few useful transforms."""
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_yaml())
for a, b in [("world", "panda_link0"), ("world", "panda_hand"), ("panda_link0", "panda_hand"),
             ("world", "birdview_optical_frame"), ("world", "agentview_optical_frame"),
             ("world", "robot0_eye_in_hand_optical_frame")]:
    try:
        t = buf.lookup_transform(a, b, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{a}->{b}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{a}->{b}: FAIL {e}")
rclpy.shutdown()
