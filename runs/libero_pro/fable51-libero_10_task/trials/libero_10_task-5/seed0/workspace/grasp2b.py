#!/usr/bin/env python3
"""Continue the tilted rim approach from the current pose (baseline wrench taken here)."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

A = np.radians(60.0)
TCP = np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv) > 3 else np.array([-0.330, -0.148, 1.065])
STEP = 0.01
z = np.array([-np.cos(A), 0, -np.sin(A)])
y = np.array([-np.sin(A), 0, np.cos(A)])
x = np.cross(y, z)
Q = Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()

r = Robot("grasp2b")
p0 = r.report("start")
r.gripper(0.04)
base_w = r.wrench()
print("baseline wrench", base_w.round(2))
d0 = float(np.dot(TCP - p0, z))            # remaining distance along approach
print("remaining approach", round(d0, 3))
seed = r.arm_q()
for d in np.arange(d0 - STEP, -1e-6, -STEP):
    p = TCP - d * z
    s = r.ik(p, Q, seed=seed); assert s, f"IK fail at d={d}"
    seed = s
    r.move(s, 0.8, retries=1)
    w = r.wrench()
    r.report(f"d={d:.3f}")
    dw = w - base_w
    print("   dwrench", dw.round(2))
    if np.abs(dw[:3]).max() > 2.5:
        print("   contact! stopping approach")
        break
f = r.gripper(0.0)
w = r.wrench()
print("after close: fingers", f, "dwrench", (w - base_w).round(2))
