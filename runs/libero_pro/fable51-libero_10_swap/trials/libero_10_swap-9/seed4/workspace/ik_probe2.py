#!/usr/bin/env python3
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe2")
q0 = r.q()
gx, gy, gz = -0.0135, -0.030, 0.980
for th in [25, 30, 35]:
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]
    for sx in [1, -1]:
        x = [0, sx*np.cos(t), sx*np.sin(t)]
        quat = quat_from_axes(z, x)
        seed = q0
        for name, tip in [("pre", (gx, gy, 1.10)), ("grasp", (gx, gy, gz)), ("lift", (gx, gy, 1.25)),
                          ("carry", (-0.13, -0.45, 1.25)), ("down", (-0.13, -0.45, 1.04)),
                          ("in", (-0.13, -0.338, 1.04)), ("rel", (-0.13, -0.338, 1.03))]:
            q = r.solve_ik(tip, quat, seed=seed, tries=8)
            if q is None:
                print(f"th={th} sx={sx} {name}: --"); continue
            seed = q
            p7, _ = r.fk_pose(q, "panda_link7")
            p6, _ = r.fk_pose(q, "panda_link6")
            ph, _ = r.fk_pose(q, "panda_hand")
            print(f"th={th} sx={sx} {name}: q={np.round(q,2)} hand={np.round(ph,3)} l7={np.round(p7,3)} l6={np.round(p6,3)}")
            sys.stdout.flush()
