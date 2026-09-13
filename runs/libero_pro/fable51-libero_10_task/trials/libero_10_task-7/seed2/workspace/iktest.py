import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
p, q, tcp, j = a.state()
import numpy as np
for label, off in [("base-frame", a.base), ("world-frame", np.zeros(3))]:
    sol = a.solve_ik(p + a.base - off if label=="world-frame" else p, q)
    # note: solve_ik subtracts base internally; for world-frame test add base back
    print(label, None if sol is None else [round(v,3) for v in sol])
# also try with the exact quaternion but z slightly different
for dz in (0.0, 0.05, -0.05):
    sol = a.solve_ik(p + [0,0,dz], (1.0,0.0,0.0,0.0))
    print("dz", dz, None if sol is None else [round(v,3) for v in sol])
rclpy.shutdown()
