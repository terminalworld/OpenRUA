#!/usr/bin/env python3
"""Step 1: open gripper, move above the mug handle, descend to grasp height (no closing yet)."""
import sys
import numpy as np
from kin import Robot

QD = np.array([0.7071, 0.7071, 0.0, 0.0])  # hand z down, fingers close along world x
HX, HY = -0.098, -0.042                     # handle top-arm centre (world)
Z_PRE, Z_GRASP = 1.10, float(sys.argv[1]) if len(sys.argv) > 1 else 0.960

r = Robot("pick1")
r.report("start")
r.gripper(0.04)

q = r.arm_q()
pre = r.ik((HX, HY, Z_PRE), QD, seed=q)
assert pre is not None, "IK pre-grasp failed"
print("pre-grasp q", np.round(pre, 3))
r.move(pre, 4.0)
r.report("pre-grasp")

# straight-line descent through IK waypoints, 2 cm apart
zs = np.arange(Z_PRE - 0.02, Z_GRASP - 1e-6, -0.02)
zs = np.append(zs, Z_GRASP)
seed = pre
via = []
for i, z in enumerate(zs):
    s = r.ik((HX, HY, z), QD, seed=seed)
    assert s is not None, f"IK failed at z={z}"
    seed = s
    via.append((s, 0.6 * (i + 1)))
last_q, last_t = via.pop()
r.move(last_q, last_t, via=via)
r.report("at grasp height")
w = r.wrench()
print("wrench", None if w is None else w.round(2))
