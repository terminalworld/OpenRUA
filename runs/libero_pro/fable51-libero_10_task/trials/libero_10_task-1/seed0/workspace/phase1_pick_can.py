#!/usr/bin/env python3
"""Pick the alphabet soup can at (-0.085, -0.14); can top z=0.502, table 0.426."""
from robolib import *

CAN = (-0.085, -0.14)
r = Robot("p1")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("open gripper"); r.gripper(GRIP["open_m"])
print("above can"); ok = r.move_to((CAN[0], CAN[1], 0.62), secs=3.0, label="above")
print("descend"); ok = r.move_to((CAN[0], CAN[1], 0.466), secs=2.5, label="grasp")
print("close"); r.gripper(GRIP["closed_m"])
gap = r.finger_gap(); print("gap after close", round(gap, 4))
print("lift"); r.move_to((CAN[0], CAN[1], 0.66), secs=2.5, label="lift")
print("gap after lift", round(r.finger_gap(), 4))
print("PHASE1 DONE")
