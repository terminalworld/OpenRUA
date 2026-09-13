#!/usr/bin/env python3
"""Phase 1: pinch white mug rim from above, lift, rotate 180 deg (joint 7),
place at staging spot with handle facing -y, release, retreat."""
import math
import sys
import numpy as np
from rob import *

M_CENTER = np.array([-0.1225, -0.2525])   # white mug centre (world xy)
RIM_Z = 1.012
S_CENTER = np.array([-0.20, -0.15])       # staging spot for mug centre
WALL_OFF = 0.044                          # TCP sits on the -y wall (radius 0.0465 - half wall)

r = Robot("phase1")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

R = R_down(0.0)                           # fingers open along world y
tcp_xy = M_CENTER + np.array([0.0, -WALL_OFF])
above = np.array([*tcp_xy, 1.12])
grasp = np.array([*tcp_xy, RIM_Z - 0.022])

log("move above mug", above)
q1, code = r.move_to_pose(above, R, seconds=4.0, seed=q0)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))

log("descend to grasp", grasp)
q2, code = r.move_to_pose(grasp, R, seconds=3.0, seed=q1)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))

gap = r.gripper(0.0)
log("closed; gap", round(gap, 4))
if gap < 0.003:
    log("!! gripper closed on air, abort"); sys.exit(2)

lift = np.array([*tcp_xy, 1.13])
log("lift", lift)
q3, code = r.move_to_pose(lift, R, seconds=3.0, seed=q2)
t, _ = r.tcp(); log("tcp now", np.round(t, 4), "gap", round(r.finger_gap(), 4))

# rotate joint 7 by pi within limits
q4 = q3.copy()
lo, hi = LIMITS[6]
if q4[6] + math.pi <= hi - 0.05:
    q4[6] += math.pi
elif q4[6] - math.pi >= lo + 0.05:
    q4[6] -= math.pi
else:
    log("!! cannot rotate joint7 by pi from", q4[6]); sys.exit(3)
log("rotate j7 ->", round(q4[6], 3))
r.move_joints([q4], [4.0])
t, Rn = r.tcp(); log("tcp now", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4))

# after rotation the mug centre is TCP + (0, -WALL_OFF); place at S
R2 = Rn.copy()
tcp2 = S_CENTER + np.array([0.0, +WALL_OFF])
over = np.array([*tcp2, 1.13])
log("move over staging", over)
q5, code = r.move_to_pose(over, R2, seconds=4.0, seed=q4)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))
down = np.array([*tcp2, RIM_Z - 0.022 + 0.004])
log("lower", down)
q6, code = r.move_to_pose(down, R2, seconds=3.0, seed=q5)
t, _ = r.tcp(); log("tcp now", np.round(t, 4))
r.gripper(0.04)
up = np.array([*tcp2, 1.15])
q7, code = r.move_to_pose(up, R2, seconds=3.0, seed=q6)
t, _ = r.tcp(); log("retreated; tcp now", np.round(t, 4))
log("PHASE1 DONE")
