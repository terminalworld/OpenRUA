#!/usr/bin/env python3
"""Dump all TF frames and world->frame transforms; also FK of panda_hand."""
import rclpy, yaml
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node("tfdump")
buf = Buffer()
TransformListener(buf, node)
frames = {}
def cb(m):
    for t in m.transforms:
        frames[t.child_frame_id] = t.header.frame_id
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.2)
print("edges (child <- parent):")
for c, p in sorted(frames.items()):
    print(f"  {c} <- {p}")
print()
for f in sorted(frames):
    try:
        t = buf.lookup_transform("world", f, Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f"world->{f}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e:
        print(f"world->{f}: FAIL {type(e).__name__}")
rclpy.shutdown()
