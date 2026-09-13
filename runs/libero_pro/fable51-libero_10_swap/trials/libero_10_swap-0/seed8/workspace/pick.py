#!/usr/bin/env python3
"""Pick an object with a tilted top-down grasp and lift it.

Usage: python3 pick.py x y z_grasp z_pre tilt_deg yaw_deg seed_csv
World frame. Seed = joint vector to seed IK with (keeps the branch).
"""
import sys
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as Rot

argv = sys.argv[1:]
sys.argv = [sys.argv[0]]
from arm import Arm, GRIP, quat_to_R, TCP

x, y, zg, zp, tilt, yaw = map(float, argv[:6])
seed = [float(v) for v in argv[6].split(",")]
q = (Rot.from_euler('z', yaw, degrees=True) * Rot.from_euler('y', tilt, degrees=True)
     * Rot.from_quat([1, 0, 0, 0])).as_quat()
a = Arm()
print("fingers before:", a.fingers())

# 1. pre-grasp
s_pre = a.ik((x, y, zp), q, seed=seed, at_tcp=True)
if s_pre is None:
    raise SystemExit("IK pregrasp failed")
print("pregrasp sol", np.round(s_pre, 3))
code, err = a.traj([s_pre], 4.0)
if err > 0.05:
    raise SystemExit("pregrasp not reached")
pos, fq = a.fk()
print("tcp now", np.round(pos + TCP * quat_to_R(fq)[:, 2], 4))

# 2. descend in steps
zs = np.linspace(zp, zg, 4)[1:]
pts, s = [], s_pre
for z in zs:
    s = a.ik((x, y, z), q, seed=s, at_tcp=True)
    if s is None:
        raise SystemExit(f"IK descend failed at z={z}")
    pts.append(s)
code, err = a.traj(pts, 3.0)
pos, fq = a.fk()
tcp = pos + TCP * quat_to_R(fq)[:, 2]
print("tcp at grasp", np.round(tcp, 4), "target", (x, y, zg))

# 3. close
f = a.gripper(GRIP["closed_m"])
gap = f[0] - f[1]
print(f"finger gap after close: {gap:.4f} m")
if gap < 0.01:
    raise SystemExit("GRASP FAILED: closed on air")

# 4. lift straight up to zp
s_lift = a.ik((x, y, zp), q, seed=s, at_tcp=True)
code, err = a.traj([s_lift], 3.0)
f = a.fingers()
print(f"lifted; finger gap now {f[0]-f[1]:.4f}")
pos, fq = a.fk()
print("tcp after lift", np.round(pos + TCP * quat_to_R(fq)[:, 2], 4))
print("joints", ",".join(f"{v:.4f}" for v in a.arm_q()))
rclpy.shutdown()
