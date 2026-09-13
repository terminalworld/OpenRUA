#!/usr/bin/env python3
"""Carry the held object to a world xy over the basket and release.

Usage: python3 place.py x y z_release tilt_deg
"""
import sys
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as Rot

argv = sys.argv[1:]
sys.argv = [sys.argv[0]]
from arm import Arm, GRIP, quat_to_R, TCP

x, y, zr, tilt = map(float, argv[:4])
q = (Rot.from_euler('y', tilt, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()
a = Arm()
f = a.fingers()
print("fingers before (gap):", f[0] - f[1])

pos, fq = a.fk()
tcp0 = pos + TCP * quat_to_R(fq)[:, 2]
mid = np.array([(tcp0[0] + x) / 2, (tcp0[1] + y) / 2, 0.80])
seed = a.arm_q()
s_mid = a.ik(mid, q, seed=seed, at_tcp=True)
if s_mid is None:
    raise SystemExit("IK mid failed")
s_end = a.ik((x, y, zr), q, seed=s_mid, at_tcp=True)
if s_end is None:
    raise SystemExit("IK release pose failed")
print("mid", np.round(s_mid, 3), "\nend", np.round(s_end, 3))
a.traj([s_mid], 4.0)
a.traj([s_end], 3.0)
pos, fq = a.fk()
tcp = pos + TCP * quat_to_R(fq)[:, 2]
print("tcp at release", np.round(tcp, 4), "target", (x, y, zr))
print("gap before release", (lambda f: f[0] - f[1])(a.fingers()))
if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.03:
    raise SystemExit("not over target; NOT releasing")
f = a.gripper(GRIP["open_m"])
print("gap after release", f[0] - f[1])
# retreat up
s_up = a.ik((x, y, 0.85), q, seed=s_end, at_tcp=True)
a.traj([s_up], 3.0)
print("joints", ",".join(f"{v:.4f}" for v in a.arm_q()))
rclpy.shutdown()
