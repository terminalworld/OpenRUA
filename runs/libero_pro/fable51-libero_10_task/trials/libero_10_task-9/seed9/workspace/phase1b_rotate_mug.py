#!/usr/bin/env python3
"""Phase 1b: re-grasp mug rim from above, lift, rotate via joint 7 so the
handle faces -y, place at S, release, retreat."""
import math
import sys
import numpy as np
from rob import *

C = np.array([-0.2201, -0.0698])      # mug centre now
HANDLE_AZ = math.radians(133.5)       # handle azimuth now
TARGET_AZ = math.radians(-90.0)       # want handle pointing -y
RIM_Z = 1.012
S = np.array([-0.21, -0.09])          # staging spot for mug centre
WALL_OFF = 0.044

alpha = (TARGET_AZ - HANDLE_AZ + math.pi) % (2 * math.pi) - math.pi   # CCW rotation needed, in (-pi, pi]
log(f"need CCW rotation alpha={math.degrees(alpha):.1f} deg")

r = Robot("phase1b")
q0 = r.arm_q()
log("start q", np.round(q0, 3), "gap", round(r.finger_gap(), 4))
if r.finger_gap() < 0.07:
    r.gripper(0.04)

# pick a pinch azimuth psi whose IK joint7 allows the rotation
lo, hi = LIMITS[6]
chosen = None
for psi_deg in (-90, 180, 0, 90):
    psi = math.radians(psi_deg)
    yaw = math.atan2(-math.cos(psi), -math.sin(psi))
    R = R_down(yaw)
    tcp_xy = C + WALL_OFF * np.array([math.cos(psi), math.sin(psi)])
    above = np.array([*tcp_xy, 1.12])
    q = r.ik(above, R, seed=q0)
    if q is None:
        log(f"psi={psi_deg}: IK failed"); continue
    dj7 = -alpha
    j7_new = q[6] + dj7
    ok = lo + 0.05 <= j7_new <= hi - 0.05
    log(f"psi={psi_deg}: j7={q[6]:.3f} -> {j7_new:.3f} ok={ok}")
    if ok:
        chosen = (psi, R, tcp_xy, q, dj7); break
    # try the equivalent rotation the other way round
    dj7b = -(alpha - 2 * math.pi) if alpha > 0 else -(alpha + 2 * math.pi)
    j7_new = q[6] + dj7b
    ok = lo + 0.05 <= j7_new <= hi - 0.05
    log(f"psi={psi_deg}: alt dj7={dj7b:.3f} -> {j7_new:.3f} ok={ok}")
    if ok:
        chosen = (psi, R, tcp_xy, q, dj7b); break
if chosen is None:
    log("!! no feasible pinch/rotation"); sys.exit(3)
psi, R, tcp_xy, q_above, dj7 = chosen

log("move above", np.round(tcp_xy, 4))
r.move_joints([q_above])
grasp = np.array([*tcp_xy, RIM_Z - 0.022])
q2, err = r.move_to_pose(grasp, R, seed=q_above)
gap = r.gripper(0.0)
if gap < 0.003:
    log("!! closed on air"); sys.exit(2)
lift = np.array([*tcp_xy, 1.13])
q3, err = r.move_to_pose(lift, R, seed=q2)
log("gap after lift", round(r.finger_gap(), 4))

q4 = q3.copy(); q4[6] += dj7
log(f"rotate j7 by {dj7:.3f} -> {q4[6]:.3f}")
r.move_joints([q4], speed=0.6)
t, Rn = r.tcp()
log("tcp", np.round(t, 4), "y_hand", np.round(Rn[:, 1], 3), "gap", round(r.finger_gap(), 4))

# actual rotation achieved (from hand y axis): planned alpha = -dj7
alpha_done = -dj7
v0 = -WALL_OFF * np.array([math.cos(psi), math.sin(psi)])         # mug centre rel. TCP before
ca, sa = math.cos(alpha_done), math.sin(alpha_done)
v = np.array([ca * v0[0] - sa * v0[1], sa * v0[0] + ca * v0[1]])  # after
tcp_place = S - v
log("place: tcp xy", np.round(tcp_place, 4), "mug centre", S)
q5, err = r.move_to_pose(np.array([*tcp_place, 1.13]), Rn, seed=q4)
q6, err = r.move_to_pose(np.array([*tcp_place, RIM_Z - 0.022 + 0.004]), Rn, seed=q5)
r.gripper(0.04)
q7, err = r.move_to_pose(np.array([*tcp_place, 1.15]), Rn, seed=q6)
log("PHASE1B DONE")
