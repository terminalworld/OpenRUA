#!/usr/bin/env python3
"""Carry the cup (held by its wall) to the caddy's left compartment,
handle pointing -x (toward the back wall), and release it just above
the floor.  Run steps one at a time: python3 place.py <step>"""
import math
import sys
import numpy as np
from rob import *

taper = math.atan2(0.0125, 0.105)
R_grasp = hand_R(math.pi / 2, [0, 1, 0], taper)   # current: cup at -x of TCP
R_place = hand_R(0.0, [1, 0, 0], taper)           # after -90 deg yaw: cup at +y of TCP
print("R_place hand z", np.round(R_place[:, 2], 3), "finger axis", np.round(R_place[:, 1], 3))

cup_xy = np.array([-0.385, -0.272])       # target cup centre in the left compartment
tcp_xy = cup_xy + np.array([0.0, -0.0447])  # grasp point is on the cup's -y wall after the yaw

step = sys.argv[1]
r = Robot()
q0 = r.arm_q()
print("q0", np.round(q0, 3), "gap %.4f" % r.finger_gap())

if step == "rotate":
    tcp = r.tcp_pose()[0]
    q = r.ik_tcp(tcp, R_place, seed=q0)
    print("ik", None if q is None else np.round(q, 3))
    if q is not None:
        print("max joint delta", np.max(np.abs(np.array(q) - q0)))
        if np.max(np.abs(np.array(q) - q0)) < 2.0:
            print(r.move_joints(q, 5.0))
            print("tcp now", r.tcp_pose()[0], "gap %.4f" % r.finger_gap())
elif step == "carry":
    print(r.move_tcp([tcp_xy[0], tcp_xy[1], 1.22], R_place, 5.0))
    print("gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "lower":
    for z in (1.08, 1.03, 1.00):
        print(z, r.move_tcp([tcp_xy[0], tcp_xy[1], z], R_place, 2.5))
        print("  gap %.4f" % r.finger_gap(), "F", np.round(r.wrench()[0], 2))
elif step == "release":
    print(r.gripper(0.08)); print(r.gripper(0.08))
    print("gap %.4f" % r.finger_gap())
    print(r.move_tcp([tcp_xy[0], tcp_xy[1], 1.25], R_place, 3.0))
    print(r.move_tcp([-0.20, -0.30, 1.30], hand_R(0.0), 4.0))
r.close()
