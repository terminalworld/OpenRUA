#!/usr/bin/env python3
"""Pick one object (top-down grasp) and drop it into the basket.
Usage: python3 -u pick_place.py <x> <y> <grasp_z> <hover_z> [xfingers]
"""
import sys
import numpy as np
from robot import Robot, Q_DOWN_Y, Q_DOWN_X

x, y, gz, hz = map(float, sys.argv[1:5])
Q = Q_DOWN_X if (len(sys.argv) > 5 and sys.argv[5] == "xfingers") else Q_DOWN_Y
BASKET = np.array([0.0, 0.25])
DROP_Z = 0.70
TRANSIT_Z = 0.75

r = Robot()
print("start tcp", r.tcp_world().round(4), "fingers", r.fingers())

print("[1] open gripper")
r.gripper(0.04)

print("[2] hover above object")
if r.move_tcp(np.array([x, y, hz]), Q, 4.0) is None:
    sys.exit("hover IK failed")

print("[3] descend to grasp height")
if r.move_tcp(np.array([x, y, gz]), Q, 2.5) is None:
    sys.exit("grasp IK failed")
w0 = r.wrench()

print("[4] close gripper")
f = r.gripper(0.0)
gap = f[0] - f[1]
print(f"  finger gap = {gap:.4f} m (0 => closed on air)")
if gap < 0.005:
    sys.exit("GRASP FAILED: fingers closed fully")

print("[5] lift")
r.move_tcp(np.array([x, y, TRANSIT_Z]), Q, 2.5)
f = r.fingers()
print(f"  after lift finger gap = {f[0]-f[1]:.4f}", "wrench", r.wrench().round(2), "was", w0.round(2))
if f[0] - f[1] < 0.005:
    sys.exit("object lost during lift")

print("[6] move over basket")
if r.move_tcp(np.array([BASKET[0], BASKET[1], TRANSIT_Z]), Q, 4.0) is None:
    sys.exit("basket IK failed")
r.move_tcp(np.array([BASKET[0], BASKET[1], DROP_Z]), Q, 2.0)
f = r.fingers()
print(f"  over basket finger gap = {f[0]-f[1]:.4f}")

print("[7] release")
r.gripper(0.04)

print("[8] retreat up")
r.move_tcp(np.array([BASKET[0], BASKET[1], TRANSIT_Z + 0.05]), Q, 2.0)
print("DONE")
