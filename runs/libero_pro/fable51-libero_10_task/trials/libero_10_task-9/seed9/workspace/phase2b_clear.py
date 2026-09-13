#!/usr/bin/env python3
"""Phase 2b: (1) carry the fallen yellow mug away by its handle; (2) move the
white mug into the insertion corridor at (-0.02, -0.25) with a rim pinch."""
import math
import sys
import numpy as np
from rob import *

r = Robot("phase2b")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

# ---------- yellow mug ----------
YH = np.array([-0.011, 0.078])        # handle top bar (runs along y), top z 1.035
Ry = R_down(math.pi / 2)              # fingers along x
log("hand y", np.round(Ry[:, 1], 3))
q1, _ = r.move_to_pose(np.array([*YH, 1.20]), Ry, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([*YH, 1.012]), Ry)
t, _ = r.tcp(); log("at yellow handle, tcp", np.round(t, 4))
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air (yellow)"); r.gripper(0.04); sys.exit(2)
r.move_line(np.array([*YH, 1.20]), Ry)
log("yellow lifted, gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! yellow slipped"); r.gripper(0.04); sys.exit(5)
YP = np.array([-0.35, -0.40])
r.move_line(np.array([*YP, 1.20]), Ry, step=0.05)
r.move_line(np.array([*YP, 0.965]), Ry)
r.gripper(0.04)
r.move_line(np.array([*YP, 1.20]), Ry)
log("yellow parked")

# ---------- white mug ----------
C = np.array([-0.222, -0.123])
S = np.array([-0.02, -0.25])
Rw = R_down(0.0)                      # fingers along y
tcp_xy = C + np.array([0.0, 0.044])   # pinch the +y rim point (handle is at -y)
r.move_line(np.array([*tcp_xy, 1.20]), Rw, step=0.05)
r.move_line(np.array([*tcp_xy, 0.985]), Rw)
t, _ = r.tcp(); log("at white rim, tcp", np.round(t, 4))
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air (white)"); r.gripper(0.04); sys.exit(2)
r.move_line(np.array([*tcp_xy, 1.07]), Rw)
log("white lifted, gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! white slipped"); r.gripper(0.04); sys.exit(5)
place = S + np.array([0.0, 0.044])
r.move_line(np.array([*place, 1.07]), Rw, step=0.05)
r.move_line(np.array([*place, 0.992]), Rw)
r.gripper(0.04)
r.move_line(np.array([*place, 1.20]), Rw)
r.move_line(np.array([-0.35, -0.20, 1.25]), Rw, step=0.05)
log("PHASE2B DONE")
