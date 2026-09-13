#!/usr/bin/env python3
"""Carry the held butter over the basket and release (offset from the can at center)."""
from robolib import *

DROP = (-0.025, 0.29)
r = Robot("p4")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("raise"); r.move_to((0.03, 0.061, 0.74), secs=2.0, label="raise")
print("above basket"); r.move_to((DROP[0], DROP[1], 0.74), secs=2.5, label="above basket")
print("lower"); r.move_to((DROP[0], DROP[1], 0.665), secs=2.0, label="lower")
print("gap before release", round(r.finger_gap(), 4))
print("open"); r.gripper(GRIP["open_m"])
print("retreat"); r.move_to((DROP[0], DROP[1], 0.82), secs=2.0, label="retreat")
print("PHASE4 DONE")
