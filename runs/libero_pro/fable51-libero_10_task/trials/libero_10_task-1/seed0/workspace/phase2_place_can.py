#!/usr/bin/env python3
"""Carry the held can over the basket center (0, 0.27) and release."""
from robolib import *

BASKET = (0.0, 0.27)
r = Robot("p2")
print("start tcp", r.tcp_pose()[0].round(3), "gap", round(r.finger_gap(), 4))
print("raise"); r.move_to((-0.085, -0.14, 0.74), secs=2.0, label="raise")
print("via"); r.move_to((-0.04, 0.06, 0.74), secs=2.5, label="via")
print("above basket"); r.move_to((BASKET[0], BASKET[1], 0.74), secs=2.5, label="above basket")
print("lower"); r.move_to((BASKET[0], BASKET[1], 0.69), secs=2.0, label="lower")
print("gap before release", round(r.finger_gap(), 4))
print("open"); r.gripper(GRIP["open_m"])
print("retreat"); r.move_to((BASKET[0], BASKET[1], 0.80), secs=2.0, label="retreat")
print("PHASE2 DONE")
