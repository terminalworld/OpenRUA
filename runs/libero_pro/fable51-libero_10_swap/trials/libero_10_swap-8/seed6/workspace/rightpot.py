#!/usr/bin/env python3
"""Rotate a knob-pinched pot about the pinch point so its axis becomes vertical (base down).
Usage: python3 -u rightpot.py <ax> <ay> [hz_pinch=0.095]
  (ax,ay): world direction from the knob toward the pot base (horizontal), from the cloud fit.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, log

a = np.array([float(sys.argv[1]), float(sys.argv[2]), 0.0]); a /= np.linalg.norm(a)
HZ = float(sys.argv[3]) if len(sys.argv) > 3 else 0.095
r = Robot("rightpot")
p, qu = r.fk()
R0 = Rot.from_quat(qu).as_matrix()
n = R0[:, 1]                                  # finger (pinch) axis
a = a - n * (a @ n); a /= np.linalg.norm(a)   # make sure a is perpendicular to the pinch axis
b = np.array([0, 0, -1.0]); b = b - n * (b @ n); b /= np.linalg.norm(b)
ang = np.arctan2(np.cross(a, b) @ n, a @ b)
dR = Rot.from_rotvec(n * ang).as_matrix()
R1 = dR @ R0
log(f"pinch axis {np.round(n,3)}; rotate {np.degrees(ang):.1f} deg; hand z after: {np.round(R1[:,2],3)}")
Gp = p + HZ * R0[:, 2]
path = r.plan_rot(r.arm_q(), Gp, R1, step_deg=6.0)
assert path is not None, "rotation plan failed"
for i in range(0, len(path), 4):
    ok = r.move_path(path[i:i + 4], 4.0)
    f = r.fingers()
    log(f"chunk ok={ok} gap {f[0]-f[1]:.4f}")
    if not ok or f[0] - f[1] < 0.004:
        log("ABORT (stall or lost pot)"); sys.exit(1)
p, qu = r.fk()
log(f"hand {np.round(p,4)} R\n{np.round(Rot.from_quat(qu).as_matrix(),3)}")
log("RIGHT DONE")
