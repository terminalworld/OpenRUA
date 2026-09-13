import sys, numpy as np
from ctl import Ctl, TCP
from scipy.spatial.transform import Rotation as Rot
c = Ctl()
seed0 = c.arm_q()
# spec: x,y,z,yaw,pitch  -> R = Ry(pitch) * Rz(yaw) * Rx(180)  (pitch about world y, applied last)
for spec in sys.argv[1:]:
    x,y,z,yaw,pitch = map(float, spec.split(","))
    R = Rot.from_euler("y", pitch, degrees=True) * Rot.from_euler("z", yaw, degrees=True) * Rot.from_euler("x", 180, degrees=True)
    q = (Rot.from_euler("z", -45, degrees=True) * R).as_quat()
    hz = R.as_matrix()[:,2]
    hp = np.array([x,y,z]) - TCP*hz
    try:
        sol = c.solve_ik(hp, q, seed=seed0)
        print(spec, "OK hand_z", np.round(hz,2), np.round(sol,3).tolist())
    except SystemExit as e:
        print(spec, "FAIL", e)
