#!/usr/bin/env python3
"""Dump TF world->panda_link0, camera frames, hand pose (via TF), joint states."""
import rclpy, yaml, numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node("scene")
buf = Buffer(); TransformListener(buf, node)
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
frames = ["panda_link0", "panda_hand", "agentview_optical_frame", "birdview_optical_frame",
          "frontview_optical_frame", "sideview_optical_frame", "robot0_eye_in_hand_optical_frame",
          "robot0_robotview_optical_frame"]
for f in frames:
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"{f:36s} xyz=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"{f:36s} FAIL {str(e)[:80]}")
if "m" in js:
    print(dict(zip(js["m"].name, [round(p, 4) for p in js["m"].position])))
print(buf.all_frames_as_string())
rclpy.shutdown()
