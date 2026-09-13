import numpy as np, sys
from rob import Robot
from step import tilt_quat
r = Robot("probe4")
q0 = r.q()
for th in [30, 35, 40]:
    quat = tilt_quat(th, -1)
    seed = q0
    for name, tip in [("above", (-0.203, -0.480, 1.02)), ("grasp", (-0.203, -0.480, 0.950)),
                      ("lift", (-0.203, -0.480, 1.20)), ("carry", (-0.162, -0.475, 1.20)),
                      ("down", (-0.162, -0.475, 1.005)), ("in", (-0.162, -0.311, 1.005)), ("rel", (-0.162, -0.311, 0.994))]:
        q = r.solve_ik(tip, quat, seed=seed, tries=8)
        if q is None:
            print(f"th={th} {name}: --"); sys.stdout.flush(); continue
        seed = q
        p7, _ = r.fk_pose(q, "panda_link7")
        print(f"th={th} {name}: q={np.round(q,2)} l7={np.round(p7,3)}"); sys.stdout.flush()
