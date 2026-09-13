#!/usr/bin/env python3
"""Print world->frame transforms for given frames (default: base, hand)."""
import sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

frames = sys.argv[1:] or ["panda_link0", "panda_hand"]
rclpy.init(); n = rclpy.create_node("tfq"); b = Buffer(); TransformListener(b, n)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if all(b.can_transform("world", f, Time()) for f in frames):
        break
for f in frames:
    t = b.lookup_transform("world", f, Time())
    tr, q = t.transform.translation, t.transform.rotation
    print(f"world->{f}: xyz=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f}) "
          f"q=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")
rclpy.shutdown()
