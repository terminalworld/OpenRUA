#!/usr/bin/env python3
"""Phase 1f: rim pinch (now with correct hand orientation), lift high,
observe handle azimuth from cameras, rotate j7, observe again, place at S."""
import math
import subprocess
import sys
import numpy as np
from rob import *

C = np.array([-0.1871, -0.1017])      # mug centre now
HANDLE_AZ = math.radians(-1.0)
TARGET_AZ = math.radians(-90.0)
GRASP_Z = 0.985
S = np.array([-0.22, -0.13])
WALL_OFF = 0.044
PSI = math.pi                          # pinch the -x rim point, fingers along x
HIGH_Z = 1.22

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi
dj7 = -alpha
log(f"rotation alpha={math.degrees(alpha):.1f} deg -> dj7={dj7:.3f}")


def handle_az(centre):
    """Handle azimuth (deg) about `centre` from agentview + birdview clouds."""
    out = []
    for cam in ("agentview", "birdview"):
        subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); P = d["xyz"][d["valid"]]
        rr = np.hypot(P[:, 0] - centre[0], P[:, 1] - centre[1])
        m = (P[:, 2] > centre[2] - 0.075) & (P[:, 2] < centre[2] + 0.01) & (rr > 0.055) & (rr < 0.095)
        H = P[m]
        if len(H) < 15:
            out.append((cam, len(H), None)); continue
        az = np.degrees(np.arctan2(H[:, 1] - centre[1], H[:, 0] - centre[0]))
        out.append((cam, len(H), round(float(np.median(az)), 1)))
    return out


r = Robot("phase1f")
q0 = r.arm_q()
if r.finger_gap() < 0.07:
    r.gripper(0.04)

yaw = math.atan2(-math.cos(PSI), -math.sin(PSI))
R = R_down(yaw)
tcp_xy = C + WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
lo, hi = LIMITS[6]
above = np.array([*tcp_xy, HIGH_Z])
q1 = r.ik(above, R, seed=q0)
if q1 is None:
    log("IK fail"); sys.exit(1)
if not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
    R = R_down(yaw + math.pi)
    q1 = r.ik(above, R, seed=q0)
    log("flipped yaw; j7", None if q1 is None else round(q1[6], 3))
    if q1 is None or not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
        log("!! j7 range problem"); sys.exit(3)
log(f"j7 {q1[6]:.3f} -> {q1[6]+dj7:.3f}; hand y", np.round(R[:, 1], 3))
r.move_joints([q1], speed=0.3, retries=6)
t, Ra = r.tcp(); log("above tcp", np.round(t, 4), "y_hand", np.round(Ra[:, 1], 3))

grasp = np.array([*tcp_xy, GRASP_Z])
q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3, retries=8, tol=0.004)
t, _ = r.tcp()
if np.linalg.norm(t - grasp) > 0.006:
    log("!! grasp pose not reached"); sys.exit(4)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); r.gripper(0.04); sys.exit(2)

q3, err = r.move_to_pose(np.array([*tcp_xy, GRASP_Z + 0.045]), R, seed=q2, speed=0.3)
log("gap after lift", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! slipped"); r.gripper(0.04); r.move_to_pose(np.array([*tcp_xy, 1.15]), R, seed=q3, speed=0.3); sys.exit(5)
q3, err = r.move_to_pose(np.array([*tcp_xy, HIGH_Z]), R, seed=q3, speed=0.3)
t, _ = r.tcp()
centre_air = np.array([t[0] - WALL_OFF * math.cos(PSI), t[1] - WALL_OFF * math.sin(PSI), t[2] + 0.028])  # rim z ~ tcp+0.028
log("in air: gap", round(r.finger_gap(), 4), "handle az", handle_az(centre_air))

q4 = q3.copy(); q4[6] += dj7
err = r.move_joints([q4], speed=0.15, retries=8)
t, Rn = r.tcp()
q_now = r.arm_q()
alpha_done = -(q_now[6] - q3[6])
v0 = -WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])
centre_air = np.array([t[0] + v[0], t[1] + v[1], t[2] + 0.028])
log(f"rotated {math.degrees(alpha_done):.1f} deg; y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4),
    "handle az", handle_az(centre_air))

tcp_place = S - v
log("place tcp xy", np.round(tcp_place, 4))
q5, err = r.move_to_pose(np.array([*tcp_place, HIGH_Z]), Rn, seed=q_now, speed=0.3)
q5, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.045]), Rn, seed=q5, speed=0.3)
q6, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.006]), Rn, seed=q5, speed=0.3)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6, speed=0.3)
q8, err = r.move_to_pose(np.array([-0.35, -0.30, 1.25]), R_down(0.0), seed=q7, speed=0.3)
log("PHASE1F DONE")
