#!/usr/bin/env python3
"""Phase 2c: open the microwave door by pinching its handle slab and pulling
along the hinge arc."""
import math
import sys
import numpy as np
from rob import *

HINGE = np.array([-0.19, 0.235])
TCP0 = np.array([0.056, 0.21])        # pinch point on the handle slab (closed door)
Z = 1.02
THETA_END = math.radians(88.0)
STEP = math.radians(10.0)


def R_theta(th):
    z = np.array([math.sin(th), math.cos(th), 0.0])
    y = np.array([math.cos(th), -math.sin(th), 0.0])
    return R_from_axes(y_hand=y, z_hand=z)


def tcp_theta(th):
    v = TCP0 - HINGE
    c, s = math.cos(-th), math.sin(-th)
    return HINGE + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


r = Robot("phase2c")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)
R0 = R_theta(0.0)
log("R0 y", np.round(R0[:, 1], 3), "z", np.round(R0[:, 2], 3), "x", np.round(R0[:, 0], 3))
q1, _ = r.move_to_pose(np.array([TCP0[0], 0.05, 1.25]), R0, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([TCP0[0], 0.05, Z]), R0)
r.move_line(np.array([TCP0[0], TCP0[1], Z]), R0, step=0.02, speed=0.1)
t, Rt = r.tcp(); log("at handle tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))
gap = r.gripper(0.0)
log("gap on handle", round(gap, 4))
if gap < 0.005:
    log("!! missed handle"); r.gripper(0.04); sys.exit(2)

th = 0.0
while th < THETA_END - 1e-6:
    th_new = min(th + STEP, THETA_END)
    p = tcp_theta(th_new)
    err = r.move_line(np.array([p[0], p[1], Z]), R_theta(th), R_to=R_theta(th_new), step=0.02, speed=0.12, retries=4)
    t, Rt = r.tcp()
    log(f"theta {math.degrees(th_new):.0f}: tcp {np.round(t,4)} gap {r.finger_gap():.4f} err {err:.4f}")
    if r.finger_gap() < 0.004:
        log("!! lost the handle"); break
    th = th_new

r.gripper(0.04)
t, Rt = r.tcp()
back = t - 0.06 * Rt[:, 2]
r.move_line(back, Rt)
r.move_line(np.array([back[0], back[1], 1.25]), Rt)
log("PHASE2C DONE")
