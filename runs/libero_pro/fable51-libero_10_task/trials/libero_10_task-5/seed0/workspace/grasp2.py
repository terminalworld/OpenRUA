#!/usr/bin/env python3
"""Re-grasp the fallen cup: tilted approach through the open rim, pinch the top wall of the rim.

Cup lies with axis along x, opening toward +x at x~-0.305, top of rim z~1.067, y centre -0.148.
Hand approaches from +x/above (tilt A deg below horizontal); fingers pinch in the x-z plane.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

A = np.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 60.0)
TCP = np.array([float(v) for v in sys.argv[2:5]]) if len(sys.argv) > 4 else np.array([-0.330, -0.148, 1.065])
BACK = 0.08

z = np.array([-np.cos(A), 0, -np.sin(A)])       # approach: toward -x and down
y = np.array([-np.sin(A), 0, np.cos(A)])        # pinch direction (finger slide)
x = np.cross(y, z)
Q = Rot.from_matrix(np.stack([x, y, z], 1)).as_quat()
print("hand z", z.round(3), "pinch", y.round(3), "quat", Q.round(4))

r = Robot("grasp2")
r.report("start")
r.gripper(0.04)
base_w = r.wrench()
print("baseline wrench", base_w.round(2))

q = r.arm_q()
pre = r.ik(TCP - BACK * z, Q, seed=q); assert pre, "IK pre failed"
r.move(pre, 5.0)
r.report("pre")

seed = pre
for d in np.arange(BACK - 0.02, -1e-6, -0.02):
    p = TCP - d * z
    s = r.ik(p, Q, seed=seed); assert s, f"IK fail at d={d}"
    seed = s
    r.move(s, 1.0, retries=1)
    w = r.wrench()
    pos = r.report(f"d={d:.2f}")
    dw = w - base_w
    print("   dwrench", dw.round(2))
    if abs(dw[2]) > 2.0 or abs(dw[0]) > 2.0:
        print("   contact! stopping approach")
        break

f = r.gripper(0.0)
w = r.wrench()
print("after close: fingers", f, "dwrench", (w - base_w).round(2))
