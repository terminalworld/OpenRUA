#!/usr/bin/env python3
"""Phase 2e: close the microwave door by pushing its outer face with closed
fingertips along the hinge arc (reverse of phase 2c)."""
import math
import sys
import numpy as np
from rob import *

H = np.array([-0.18, 0.235])          # hinge (refined from the open-door slab measurement)
RHO = 0.16                            # push point distance from the hinge (closed x ~ -0.02, clear of the handle)
T = 0.015                             # half slab thickness (outer face offset from the door centreline)
CLEAR = 0.008                         # fingertips this far outside the modelled face
Z = 1.01
TH0 = math.radians(86.0)              # door currently open this much
TH_END = math.radians(-3.0)           # push a little past closed
STEP = math.radians(8.0)


def R_theta(th):
    z = np.array([math.sin(th), math.cos(th), 0.0])     # fingertips point at the door face
    y = np.array([math.cos(th), -math.sin(th), 0.0])    # hand width along the door
    return R_from_axes(y_hand=y, z_hand=z)


def push_pt(th, c=CLEAR):
    d = np.array([math.cos(th), -math.sin(th)])
    n = np.array([-math.sin(th), -math.cos(th)])
    p = H + RHO * d + (T + c) * n
    return np.array([p[0], p[1], Z])


r = Robot("phase2e")
q0 = r.arm_q()
r.gripper(0.0)
t0, R0 = r.tcp(); log("start tcp", np.round(t0, 4))

# 1. back away from the microwave front, then swing to the approach pose beside the door's free end
r.move_line(np.array([t0[0], -0.12, 1.15]), R0, step=0.03, speed=0.15)
Ra = R_theta(TH0)
q1, _ = r.move_to_pose(np.array([-0.25, -0.12, Z]), Ra, seed=r.arm_q(), speed=0.25, retries=6)
t, Rt = r.tcp(); log("approach tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))

# 2. slide along the outer side of the open door, then in to the face
r.move_line(np.array([-0.25, push_pt(TH0)[1], Z]), Ra, step=0.02, speed=0.1)
r.move_line(push_pt(TH0), Ra, step=0.015, speed=0.06)
t, _ = r.tcp(); log("at door face tcp", np.round(t, 4), "model", np.round(push_pt(TH0), 4))

# 3. push along the arc
th = TH0
while th > TH_END + 1e-6:
    th_new = max(th - STEP, TH_END)
    err = r.move_line(push_pt(th_new), R_theta(th), R_to=R_theta(th_new), step=0.02, speed=0.1, retries=4)
    t, _ = r.tcp()
    log(f"theta {math.degrees(th_new):.0f}: tcp {np.round(t, 4)} err {err:.4f}")
    if err > 0.05:
        log("!! door seems jammed; stopping the push"); break
    th = th_new

# 4. retreat straight back (-z_hand) and up
t, Rt = r.tcp()
back = t - 0.07 * Rt[:, 2]
r.move_line(back, Rt, step=0.02, speed=0.1)
r.move_line(np.array([back[0], back[1] - 0.05, 1.20]), Rt, step=0.03, speed=0.15)
r.gripper(0.04)
log("PHASE2E DONE")
