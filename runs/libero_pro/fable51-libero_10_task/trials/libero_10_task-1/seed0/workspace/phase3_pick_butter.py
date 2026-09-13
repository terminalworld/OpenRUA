#!/usr/bin/env python3
"""Pick the butter at (0.030, 0.061); top z=0.454, table 0.426. Fingers along y (3.5 cm side)."""
from robolib import *

B = (0.030, 0.061)
r = Robot("p3")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("above butter"); r.move_to((B[0], B[1], 0.60), secs=3.0, label="above")
print("pre-grasp"); r.move_to((B[0], B[1], 0.48), secs=2.0, label="pre", tol=0.01)
print("descend"); r.move_to((B[0], B[1], 0.437), secs=2.0, label="grasp", tol=0.005)
print("close"); r.gripper(GRIP["closed_m"])
print("gap after close", round(r.finger_gap(), 4))
print("lift"); r.move_to((B[0], B[1], 0.60), secs=2.5, label="lift")
print("gap after lift", round(r.finger_gap(), 4))
print("PHASE3 DONE")
