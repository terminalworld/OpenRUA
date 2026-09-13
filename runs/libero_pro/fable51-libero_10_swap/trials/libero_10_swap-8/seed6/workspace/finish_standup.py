#!/usr/bin/env python3
"""Finish righting a pot already held horizontally in the gripper (top-down neck grasp).
Usage: python3 -u finish_standup.py <phi2_deg> <d_eff> [hz_pad=0.105]
  phi2_deg: azimuth the pot axis (base->lid) is yawed to before righting; the hand's -x axis
            currently points to the lid.
  d_eff:    distance from the pad contact point to the pot base (m).
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from geom import R_PLACE, Ry, Rz
from rob import Robot, log

phi2 = np.radians(float(sys.argv[1]))
d_eff = float(sys.argv[2])
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.105
TABLE_Z = 0.90
FLIP = np.diag([-1.0, -1.0, 1.0])
u2 = np.array([np.cos(phi2), np.sin(phi2), 0.0])

r = Robot("finish")


def hand_R():
    p, qu = r.fk()
    return p, Rot.from_quat(qu).as_matrix()


p, R = hand_R()
log(f"hand {np.round(p,4)} x {np.round(R[:,0],3)} z {np.round(R[:,2],3)} fingers {r.fingers()}")
# pick the flip so that the hand's -x axis (lid direction) matches u2 after the yaw
R_G2 = Rz(phi2) @ R_PLACE @ FLIP
if np.dot(R_G2[:, 0], -u2) < 0.9:
    R_G2 = Rz(phi2) @ R_PLACE
assert np.dot(R_G2[:, 0], -u2) > 0.9, "flip logic"
dR = Rz(phi2) @ Ry(np.radians(-90)) @ Rz(-phi2)   # -90deg about n2: maps u2 -> +z, -z -> +u2
R_F2 = dR @ R_G2
log(f"R_F2 z {np.round(R_F2[:,2],3)} (should be u2 {np.round(u2,3)}), x {np.round(R_F2[:,0],3)} (should be +z)")

# 1. yaw about the vertical through the pad point, in chunks
Gp = p + HZ * R[:, 2]
yaw = r.plan_rot(r.arm_q(), Gp, R_G2, step_deg=8.0)
assert yaw is not None, "yaw plan failed"
log(f"yaw: {len(yaw)} steps")
for i in range(0, len(yaw), 3):
    chunk = yaw[i:i + 3]
    ok = r.move_path(chunk, 6.0)
    log(f"yaw chunk ok={ok} fingers {r.fingers()}")
    if not ok:
        log("yaw stalled; abort"); sys.exit(1)

# 2. right the pot: rotate about the pad point
p, R = hand_R()
Gp = p + HZ * R[:, 2]
rot = r.plan_rot(r.arm_q(), Gp, R_F2, step_deg=6.0)
assert rot is not None, "rot plan failed"
for i in range(0, len(rot), 3):
    ok = r.move_path(rot[i:i + 3], 6.0)
    log(f"rot chunk ok={ok} fingers {r.fingers()}")
    if not ok:
        log("rotation stalled; abort"); sys.exit(1)
p, R = hand_R()
log(f"after rotation hand {np.round(p,4)} z {np.round(R[:,2],3)} x {np.round(R[:,0],3)}")

# 3. lower until the base is 3 mm above the table (pad point z = table + d_eff + 0.003)
Gp = p + HZ * R[:, 2]
p_down = p + np.array([0, 0, TABLE_Z + d_eff + 0.003 - Gp[2]])
down = r.plan_line(r.arm_q(), p_down, R)
assert down is not None, "down plan failed"
ok = r.move_path(down, 6.0)
log(f"down ok={ok} fingers {r.fingers()}")
r.gripper(0.04)
p, R = hand_R()
ret = r.plan_line(r.arm_q(), p - 0.07 * u2 + [0, 0, 0.10], R)
if ret is not None:
    r.move_path(ret, 4.0)
log("FINISH DONE")
