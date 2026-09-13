#!/usr/bin/env python3
"""Rim pinch on the cup's +x wall with the finger pads tilted to match the
cup's wall taper (rim r 5.0 cm -> base r 3.75 cm over 10.5 cm ~ 6.8 deg)."""
import math, sys
import numpy as np
from rob import *

cx, cy = float(sys.argv[1]), float(sys.argv[2])   # rim circle centre (birdview fit)
rim_z = 0.990
taper = math.atan2(0.0125, 0.105)   # ~6.8 deg
depth = 0.025                       # pinch this far below the rim
outer_r = 0.050 - depth * math.tan(taper)
wall_x = cx + outer_r - 0.0023      # wall centreline

R = hand_R(math.pi / 2, tilt_axis=[0, 1, 0], tilt=taper)   # hand z leans toward -x
print("hand z axis", R[:, 2], "finger axis (hand y)", R[:, 1])
r = Robot()
print("open", r.gripper(0.08))
pre = [wall_x, cy, 1.15]
grasp = [wall_x, cy, rim_z - depth]
print("pre", pre, r.move_tcp(pre, R, 4.0))
print("grasp", grasp, r.move_tcp(grasp, R, 3.0))
print("tcp", r.tcp_pose()[0], "wrench", r.wrench()[0])
for i in range(3):
    print("close", r.gripper(0.0), "gap", r.finger_gap())
print("wrench", r.wrench()[0])
r.close()
