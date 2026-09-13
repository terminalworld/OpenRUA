#!/usr/bin/env python3
"""Dump all TF frames and their transform relative to world (if connected)."""
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = yaml.safe_load(buf.all_frames_as_yaml()) or {}
for f, info in sorted(frames.items()):
    parent = info["parent"]
    line = f"{f:32s} parent={parent:20s}"
    for ref in ("world", "panda_link0"):
        try:
            t = buf.lookup_transform(ref, f, Time())
            tr, q = t.transform.translation, t.transform.rotation
            line += f" | {ref}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})"
        except Exception as e:
            line += f" | {ref}: n/a"
    print(line)
rclpy.shutdown()
