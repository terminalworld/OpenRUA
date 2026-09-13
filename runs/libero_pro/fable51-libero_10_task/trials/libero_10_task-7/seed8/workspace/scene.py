#!/usr/bin/env python3
"""Dump the full TF tree once and the hand pose via FK. Also expose helpers."""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ARM = [f"panda_joint{i}" for i in range(1, 8)]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("scene")
    tfs = {}
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def cb(msg):
        for t in msg.transforms:
            tfs[(t.header.frame_id, t.child_frame_id)] = t.transform
    node.create_subscription(TFMessage, "/tf", cb, 10)
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)
    for (p, c), t in sorted(tfs.items()):
        tr, q = t.translation, t.rotation
        print(f"{p:>28s} -> {c:<28s} t=({tr.x:+.4f},{tr.y:+.4f},{tr.z:+.4f}) q=({q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f})")

    # FK for the hand
    cli = node.create_client(GetPositionFK, "/compute_fk")
    if cli.wait_for_service(timeout_sec=10) and "m" in js:
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        for n, p in zip(js["m"].name, js["m"].position):
            if n in ARM:
                seed.name.append(n); seed.position.append(p)
        req.robot_state.joint_state = seed
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        r = fut.result()
        if r and r.error_code.val == 1:
            ps = r.pose_stamped[0]
            p, q = ps.pose.position, ps.pose.orientation
            print(f"FK hand in frame '{ps.header.frame_id}': p=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f}) q=({q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f})")
            R = quat_to_R(q.x, q.y, q.z, q.w)
            print("hand Z axis (approach) in that frame:", np.round(R[:, 2], 4))
        else:
            print("FK failed", r.error_code.val if r else None)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
