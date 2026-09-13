import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
seed = c.arm_q()
for spec in sys.argv[1:]:
    x,y,z,yaw = map(float, spec.split(","))
    R = Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hp = np.array([x,y,z]) - TCP*R.as_matrix()[:,2]
    try:
        sol = c.solve_ik(hp, q, seed=seed)
        print(spec, "OK", np.round(sol,3).tolist()); seed = sol
    except SystemExit as e:
        print(spec, "FAIL", e)
