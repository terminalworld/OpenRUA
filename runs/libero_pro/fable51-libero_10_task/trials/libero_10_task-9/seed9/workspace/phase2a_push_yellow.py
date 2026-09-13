#!/usr/bin/env python3
"""Push the fallen yellow mug +x out of the approach corridor."""
import numpy as np
from rob import *
r = Robot("push")
q0 = r.arm_q()
r.gripper(0.0)
R = R_from_axes(y_hand=np.array([0.0, 1.0, 0.0]), z_hand=np.array([0.0, 0.0, -1.0]))   # fingers along y, thin side along x
start = np.array([-0.105, 0.11, 1.15])
q1, _ = r.move_to_pose(start, R, seed=q0, speed=0.3, retries=6)
q2, _ = r.move_to_pose(np.array([-0.105, 0.11, 0.95]), R, seed=q1, speed=0.3, retries=6)
q3, _ = r.move_to_pose(np.array([0.09, 0.11, 0.95]), R, seed=q2, speed=0.15, retries=8)
q4, _ = r.move_to_pose(np.array([0.09, 0.11, 1.15]), R, seed=q3, speed=0.3, retries=6)
q5, _ = r.move_to_pose(np.array([-0.35, -0.30, 1.25]), R_down(0.0), seed=q4, speed=0.3, retries=6)
r.gripper(0.04)
log("PUSH DONE")
