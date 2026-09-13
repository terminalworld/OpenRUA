#!/usr/bin/env python3
"""Pick object at (x,y) grasping at TCP height gz, drop into basket.
Usage: python3 pick_place.py X Y GZ [yaw_deg]  (yaw rotates the fingers about world z)
"""
import sys
sys.argv_saved = list(sys.argv)
from arm import *

x, y, gz = map(float, sys.argv[1:4])
yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
BASKET = (0.0, 0.255)
HOVER = 0.66          # TCP hover above the objects
CARRY = 0.80          # TCP height while carrying (basket rim 0.628, ketchup ~13cm)
q = quat_mul((0.0, 0.0, np.sin(np.radians(yaw) / 2), np.cos(np.radians(yaw) / 2)), DOWN_Q)

a = Arm()
a.state()
print("== open gripper"); a.gripper(True)
print("== hover"); assert a.move_tcp((x, y, HOVER), 3.0, q), "hover failed"
print("== descend"); assert a.move_tcp((x, y, gz), 2.5, q), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
print(f"grasp gap={gap:.4f}")
if gap < 0.005:
    print("GRASP FAILED: closed on air"); a.gripper(True); a.move_tcp((x, y, HOVER), 2.5, q); sys.exit(2)
print("== lift"); assert a.move_tcp((x, y, CARRY), 2.5, q), "lift failed"
j = a.joints(); print("fingers after lift:", round(j["panda_finger_joint1"], 4), round(j["panda_finger_joint2"], 4))
print("== to basket"); assert a.move_tcp((BASKET[0], BASKET[1], CARRY), 3.5, q), "basket move failed"
print("== lower a bit"); a.move_tcp((BASKET[0], BASKET[1], 0.74), 2.0, q)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp((BASKET[0], BASKET[1], CARRY), 2.0, q)
a.state()
print("DONE")
rclpy.shutdown()
