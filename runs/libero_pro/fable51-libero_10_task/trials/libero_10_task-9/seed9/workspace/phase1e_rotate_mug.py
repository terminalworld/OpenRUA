#!/usr/bin/env python3
"""Phase 1c: pinch white mug rim at its -x point, lift a little, rotate via
joint 7 (slowly, fully converging) so the handle faces -y, place at S."""
import math
import sys
import numpy as np
from rob import *

C = np.array([-0.176, -0.070])      # mug centre now
HANDLE_AZ = math.radians(0.5)
TARGET_AZ = math.radians(-90.0)
RIM_Z = 1.013
GRASP_Z = 0.985
S = np.array([-0.22, -0.13])          # staging spot for mug centre
WALL_OFF = 0.044
PSI = math.pi                          # pinch at the -x rim point, fingers along x

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi
dj7 = -alpha
log(f"rotation alpha={math.degrees(alpha):.1f} deg -> dj7={dj7:.3f}")

r = Robot("phase1e")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

yaw = math.atan2(-math.cos(PSI), -math.sin(PSI))
R = R_down(yaw)
log("hand y axis (finger opening):", np.round(R[:, 1], 3))
tcp_xy = C + WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
lo, hi = LIMITS[6]

above = np.array([*tcp_xy, 1.22])
q1 = r.ik(above, R, seed=q0)
if q1 is None:
    log("IK fail above"); sys.exit(1)
if not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
    # try other yaw branch (rotate hand 180 deg about z: same pinch, fingers swapped)
    R = R_down(yaw + math.pi)
    q1 = r.ik(above, R, seed=q0)
    log("using flipped yaw; j7", None if q1 is None else round(q1[6], 3))
    if q1 is None or not (lo + 0.05 <= q1[6] + dj7 <= hi - 0.05):
        log("!! j7 range problem"); sys.exit(3)
log(f"j7 {q1[6]:.3f} -> {q1[6]+dj7:.3f}")
r.move_joints([q1], speed=0.3)
t, _ = r.tcp(); log("above tcp", np.round(t, 4))

grasp = np.array([*tcp_xy, GRASP_Z])
q2, err = r.move_to_pose(grasp, R, seed=q1, speed=0.3, retries=8, tol=0.004)
t, _ = r.tcp()
if np.linalg.norm(t - grasp) > 0.006:
    log("!! grasp pose not reached, abort"); sys.exit(4)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); r.gripper(0.04); sys.exit(2)

lift = np.array([*tcp_xy, GRASP_Z + 0.045])
high = np.array([*tcp_xy, 1.22])
q3, err = r.move_to_pose(lift, R, seed=q2, speed=0.3)
log("gap after lift", round(r.finger_gap(), 4))
if r.finger_gap() < 0.003:
    log("!! mug slipped out; release and retreat"); r.gripper(0.04)
    r.move_to_pose(np.array([*tcp_xy, 1.15]), R, seed=q3, speed=0.3); sys.exit(5)

q3, err = r.move_to_pose(high, R, seed=q3, speed=0.3)
log("gap at high", round(r.finger_gap(), 4))
q4 = q3.copy(); q4[6] += dj7
log(f"rotate j7 -> {q4[6]:.3f}")
err = r.move_joints([q4], speed=0.15, retries=8)
t, Rn = r.tcp()
log("after rotation tcp", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4), "err", round(err, 4))

# actual rotation from achieved joint7
q_now = r.arm_q()
alpha_done = -(q_now[6] - q3[6])
v0 = -WALL_OFF * np.array([math.cos(PSI), math.sin(PSI)])
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])
tcp_place = S - v
log(f"alpha_done={math.degrees(alpha_done):.1f} deg; place tcp xy {np.round(tcp_place,4)}")
q5, err = r.move_to_pose(np.array([*tcp_place, 1.22]), Rn, seed=q_now, speed=0.3)
q5, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.045]), Rn, seed=q5, speed=0.3)
q6, err = r.move_to_pose(np.array([*tcp_place, GRASP_Z + 0.006]), Rn, seed=q5, speed=0.3)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6, speed=0.3)
log("PHASE1E DONE")
