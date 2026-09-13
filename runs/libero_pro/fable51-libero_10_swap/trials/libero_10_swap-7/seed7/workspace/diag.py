#!/usr/bin/env python3
"""At the blocked pose: which joint can't reach its command? Any contact force?"""
import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *

SOUP = np.array([-0.2043, -0.1178])
Q_SOUP = q_grasp(np.pi / 2, np.radians(30))
LIM = np.array(FJT["limits_rad"])

a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f, t = w["m"].wrench.force, w["m"].wrench.torque
    return np.round([f.x, f.y, f.z], 1), np.round([t.x, t.y, t.z], 1)


a.gripper(GRIP["open_m"])
q = np.array(a.arm_q())
print("joints now     ", np.round(q, 3))
print("dist to limits ", np.round(np.minimum(q - LIM[:, 0], LIM[:, 1] - q), 3))
print("wrench now     ", wrench())
tgt = a.solve_ik([*SOUP, 0.46], q=Q_SOUP, seed=list(q))
print("IK target      ", np.round(tgt, 3) if tgt else None)
if tgt:
    print("target-now     ", np.round(np.array(tgt) - q, 3))
    print("tgt dist limits", np.round(np.minimum(np.array(tgt) - LIM[:, 0], LIM[:, 1] - np.array(tgt)), 3))
p, _ = a.hand_pose()
a.move_to([p[0], p[1], 0.62], q=Q_SOUP, seconds=3, steps=2)
print("wrench at hover", wrench())
