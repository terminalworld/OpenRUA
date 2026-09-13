#!/usr/bin/env python3
import numpy as np, sys
from rob import Robot, quat_from_axes
r = Robot("probe")
q0 = r.q()
print("q0", np.round(q0, 3))
mc = np.array([-0.0177, 0.0365])
res = {}
for th in [40, 50, 60, 70]:
    t = np.radians(th)
    z = [0, np.sin(t), -np.cos(t)]      # hand z tilted from -world z toward +y (hand leans toward -y)
    for sx in [1, -1]:
        x = [0, sx*np.cos(t), sx*np.sin(t)]  # hand x in y-z plane -> hand y = +-world x
        quat = quat_from_axes(z, x)
        for name, tip in [("pick", (mc[0], mc[1]-0.064, 1.03)),
                          ("place", (-0.15, -0.31, 1.03)),
                          ("place2", (-0.13, -0.29, 1.02))]:
            q = r.solve_ik(tip, quat, seed=q0, tries=8)
            ok = q is not None
            print(f"th={th} sx={sx} {name}: {'OK' if ok else '--'}", np.round(q,2) if ok else "")
            sys.stdout.flush()
