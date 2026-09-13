#!/usr/bin/env python3
"""Print hand and TCP pose in the WORLD frame plus finger positions.

Uses TF panda_link0->panda_hand (the arm chain is disconnected from world,
so world->base offset comes from machine facts observed via tf2_echo).
"""
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])
TCP = 0.1034


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("tcp_pose")
    buf = Buffer()
    TransformListener(buf, node)
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.__setitem__("m", m), 1)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in js and buf.can_transform("panda_link0", "panda_hand", Time()):
            break
    t = buf.lookup_transform("panda_link0", "panda_hand", Time())
    tr, q = t.transform.translation, t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    hand = np.array([tr.x, tr.y, tr.z]) + WORLD_T_BASE
    tcp = hand + TCP * R[:, 2]
    print(f"hand_world  {hand[0]:.4f} {hand[1]:.4f} {hand[2]:.4f}")
    print(f"tcp_world   {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}")
    print(f"quat xyzw   {q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}")
    print(f"finger_axis_world {R[:, 1].round(3)}")
    m = js["m"]
    d = dict(zip(m.name, m.position))
    print("joints", [round(d[f"panda_joint{i}"], 3) for i in range(1, 8)])
    print("fingers", round(d["panda_finger_joint1"], 4), round(d["panda_finger_joint2"], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
