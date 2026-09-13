#!/usr/bin/env python3
"""Carry the held mug over the plate, lower it onto the plate, release, retreat."""
import numpy as np, rclpy
from arm import Arm, DOWN_Y

PLATE = np.array([0.123, -0.005])
PLATE_TOP = 0.444
MUG_OFF = np.array([0.0, -0.036])   # mug center relative to TCP (measured)
BOTTOM_BELOW_TCP = 0.1016           # mug bottom = tcp z - this (measured)

tcp_xy = PLATE - MUG_OFF
place_z = PLATE_TOP + BOTTOM_BELOW_TCP + 0.003

a = Arm()
print("fingers", a.fingers())
print("== up to transport height")
a.goto([-0.223, 0.067, 0.72], DOWN_Y, seconds=2.5)
print("== via midpoint")
a.goto([-0.05, 0.05, 0.74], DOWN_Y, seconds=3.0)
print("== hover over plate")
a.goto([*tcp_xy, 0.72], DOWN_Y, seconds=3.0)
print("fingers", a.fingers())
print("== lower onto plate, target tcp z", round(place_z, 4))
a.goto([*tcp_xy, place_z], DOWN_Y, seconds=3.5)
print("== release")
a.gripper(0.04)
print("== retreat")
a.goto([*tcp_xy, 0.70], DOWN_Y, seconds=3.0)
rclpy.shutdown()
