#!/usr/bin/env python3
"""Phase 2d: grasp the white mug's handle with a horizontal hand from -y,
lift, check tilt, slide it into the microwave cavity, release, retreat."""
import math
import subprocess
import sys
import numpy as np
from rob import *

C = np.array([-0.0218, -0.2426])          # mug centre (rim fit)
BAR_X = -0.021                            # handle top bar x
TIP_Y = -0.300                            # fingertip y at grasp
GRASP_Z = 0.980
LIFT = 0.058                              # -> mug bottom ~0.958
TARGET_C_Y = 0.32                         # mug centre y inside cavity
OFF = C[1] - TIP_Y                        # TCP->mug centre offset along y

PHI = math.radians(40.0)                  # hand pitched down 40 deg (reachability)
R = R_from_axes(y_hand=np.array([1.0, 0.0, 0.0]), z_hand=np.array([0.0, math.cos(PHI), -math.sin(PHI)]))
log("R y", np.round(R[:, 1], 3), "z", np.round(R[:, 2], 3), "x", np.round(R[:, 0], 3))
GRASP = np.array([BAR_X, TIP_Y, GRASP_Z])
# points further back along -z_hand are unreachable (wrist too far from the base), but the
# whole column above the grasp is reachable: approach from above, fingers straddling the bar in x
PRE = np.array([BAR_X, TIP_Y, 1.04])


def mug_extents(tag):
    subprocess.run(["python3", "cloud.py", "agentview"], capture_output=True)
    d = np.load("agentview_cloud.npz"); P = d["xyz"][d["valid"]]; K = d["rgb"][d["valid"]]
    t, _ = r.tcp()
    cx, cy = t[0], t[1] + OFF
    rr = np.hypot(P[:, 0] - cx, P[:, 1] - cy)
    m = (rr < 0.06) & (P[:, 2] > 0.93) & (P[:, 2] < t[2] + 0.15) & (P[:, 1] > t[1] + 0.02)   # z>0.93 excludes the table
    Q = P[m]
    if len(Q) < 20:
        log(tag, "mug not seen", len(Q)); return None
    log(f"{tag}: mug pts {len(Q)} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}] z[{Q[:,2].min():.3f},{Q[:,2].max():.3f}] (tcp z {t[2]:.3f})")
    return Q


r = Robot("phase2d")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

log("pre", np.round(PRE, 4))
q1, _ = r.move_to_pose(np.array([-0.10, -0.36, 1.12]), R, seed=q0, speed=0.3, retries=6)
r.move_line(np.array([-0.10, TIP_Y, 1.10]), R, step=0.03, speed=0.1)
r.move_line(PRE, R, step=0.03, speed=0.1)
r.move_line(GRASP, R, step=0.015, speed=0.06, retries=8, tol=0.005)
t, Rt = r.tcp(); log("at handle tcp", np.round(t, 4), "z_hand", np.round(Rt[:, 2], 3))
gap = r.gripper(0.0)
log("gap on mug handle", round(gap, 4))
if gap < 0.004:
    log("!! missed handle"); r.gripper(0.04); r.move_line(PRE, R); sys.exit(2)

r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z + LIFT]), R, speed=0.08)
log("lifted; gap", round(r.finger_gap(), 4))
Q = mug_extents("lifted")
if r.finger_gap() < 0.004 or Q is None or Q[:, 2].max() < 1.05 or Q[:, 2].max() > 1.08:
    log("!! bad hold (slipped or tilted); putting it back")
    r.move_line(np.array([BAR_X, TIP_Y, GRASP_Z]), R, speed=0.08)
    r.gripper(0.04)
    r.move_line(PRE, R)
    sys.exit(5)

# transport: straight +y into the cavity
z = GRASP_Z + LIFT
r.move_line(np.array([BAR_X, 0.10, z]), R, step=0.04, speed=0.12)
Q = mug_extents("in front of opening")
if Q is not None and (Q[:, 2].min() < 0.947 or Q[:, 2].max() > 1.08):
    log("!! mug too tall/low for the opening; abort before insertion"); sys.exit(6)
tip_target_y = TARGET_C_Y - OFF
r.move_line(np.array([BAR_X, tip_target_y, z]), R, step=0.02, speed=0.08)
t, _ = r.tcp(); log("inserted; tcp", np.round(t, 4), "gap", round(r.finger_gap(), 4))
zr = z - 0.012                                                            # mug bottom ~2 mm above the 0.944 floor
r.move_line(np.array([BAR_X, tip_target_y, zr]), R, speed=0.05)
r.gripper(0.04)
r.move_line(np.array([BAR_X, 0.10, zr]), R, step=0.03, speed=0.1)
r.move_line(np.array([BAR_X, 0.05, 1.15]), R)
log("PHASE2D DONE")
