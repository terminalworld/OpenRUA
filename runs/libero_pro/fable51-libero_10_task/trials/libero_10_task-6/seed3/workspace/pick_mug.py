#!/usr/bin/env python3
"""Rim-grasp the red mug: hover -> descend -> close -> lift, verifying each step."""
import numpy as np, rclpy
from arm import Arm, DOWN_Y

GRASP = np.array([-0.223, 0.067])   # +y rim of the red mug (world)
RIM_Z = 0.547
HOVER_Z = 0.65
GRASP_Z = RIM_Z - 0.03

a = Arm()
print("fingers before", a.fingers())
print("== hover")
a.goto([*GRASP, HOVER_Z], DOWN_Y, seconds=4.0)
print("== descend")
a.goto([*GRASP, GRASP_Z], DOWN_Y, seconds=3.0)
print("== close")
f = a.gripper(0.0)
print("== lift")
a.goto([*GRASP, HOVER_Z], DOWN_Y, seconds=3.0)
print("fingers after lift", a.fingers())
rclpy.shutdown()
