#!/usr/bin/env python3
"""Dump TF frames and selected transforms relative to world."""
import sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node("tf_dump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = buf.all_frames_as_yaml()
print(frames)
targets = sys.argv[1:] or ["panda_link0", "panda_hand", "agentview_optical_frame",
                           "frontview_optical_frame", "birdview_optical_frame",
                           "sideview_optical_frame", "robot0_robotview_optical_frame",
                           "robot0_eye_in_hand_optical_frame"]
for f in targets:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"world->{f}: FAIL {type(e).__name__}: {e}")
rclpy.shutdown()
