#!/usr/bin/env python3
"""Clamp the (now inverted, tilted) cup's handle loop lengthwise.

Usage: grasp4.py approach|close|lift
  approach: open, go to pre-pose and creep to loop centre c (wrench-monitored), then stop (no close)
  close:    close gripper and report
  lift:     lift 12 cm straight up and report
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from kin import Robot

c = np.array([-0.350, -0.222, 1.010])           # loop centre (world)
a = np.array([-0.65, 0, 0.76]); a /= np.linalg.norm(a)   # loop long axis (= cup axis, bottom up/back)
z = np.array([-a[2], 0, a[0]])                  # approach: perpendicular to a, toward -x and down
x = np.cross(a, z)
Q = Rot.from_matrix(np.stack([x, a, z], 1)).as_quat()
BACK, STEP = 0.08, 0.01
SEED = [-0.76, 0.59, 0.45, -1.77, -0.44, 1.51, 2.23]

r = Robot("grasp4")
mode = sys.argv[1]
r.report("start")
if mode == "approach":
    print("hand z", z.round(3), "pinch", a.round(3))
    r.gripper(0.04)
    pre = r.ik(c - BACK * z, Q, seed=SEED); assert pre, "IK pre"
    r.move(pre, 5.0, retries=4)
    r.report("pre")
    base_w = r.wrench(); print("baseline wrench", base_w.round(2))
    seed = pre
    for d in np.arange(BACK - STEP, -1e-6, -STEP):
        s = r.ik(c - d * z, Q, seed=seed); assert s, f"IK d={d}"
        seed = s
        r.move(s, 0.8, retries=1)
        w = r.wrench(); dw = w - base_w
        r.report(f"d={d:.3f}")
        print("   dwrench", dw.round(2))
        if np.abs(dw[:3]).max() > 2.0:
            print("   contact! stopping approach"); break
elif mode == "close":
    w0 = r.wrench()
    f = r.gripper(0.0)
    print("after close: fingers", np.round(f, 4), "gap", round(f[0] - f[1], 4), "dwrench", (r.wrench() - w0).round(2))
elif mode == "lift":
    w0 = r.wrench()
    p, quat, R = r.tcp()
    s = r.ik(p + [0, 0, 0.12], quat, seed=r.arm_q()); assert s
    r.move(s, 3.0)
    r.report("lifted")
    print("fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
