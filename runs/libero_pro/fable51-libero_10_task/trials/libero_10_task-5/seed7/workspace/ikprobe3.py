import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl(); seed = c.arm_q()
for spec in sys.argv[1:]:
    x,y,z,yaw,pitch = map(float, spec.split(","))
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (R * Rot.from_euler("z", 45, degrees=True)).as_quat()
    hz = R.as_matrix()[:,2]; hp = np.array([x,y,z]) - TCP*hz
    try:
        sol = c.solve_ik(hp, q, seed=seed); print(spec, "OK", np.round(sol,2).tolist()); seed = sol
    except SystemExit as e: print(spec, "FAIL", e)
