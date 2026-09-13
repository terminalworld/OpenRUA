#!/usr/bin/env python3
"""Step the tilted gripper down over the soup can, logging wrench + pose."""
import subprocess

import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *

SOUP = np.array([-0.2100, -0.1310])
TILT = np.radians(30)
Q_SOUP = quat_mul((0.0, np.sin(TILT / 2), 0.0, np.cos(TILT / 2)), Q_DOWN)

a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f = w["m"].wrench.force
    return np.round([f.x, f.y, f.z], 1)


a.gripper(GRIP["open_m"])
for z in (0.62, 0.55, 0.53, 0.51, 0.49, 0.47, 0.455):
    ok = a.move_to([*SOUP, z], q=Q_SOUP, seconds=3, steps=2)
    print(f"z={z} ok={ok} wrench={wrench()} fingers={np.round(a.finger_gap(), 4)}")
    subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", f"eih_{z}.png"],
                   capture_output=True)
    if not ok:
        break
