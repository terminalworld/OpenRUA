import numpy as np
from arm import Arm
a = Arm()
q0 = a.arm_q(); print("q", np.round(q0,3))
xyz,quat = a.fk(); print("hand world", np.round(xyz,4), np.round(quat,3))
q = a.ik(xyz, quat, at_tcp=False); print("IK of current hand pose ->", None if q is None else np.round(q,3))
for cand in [(0.7071068, 0.7071068, 0.0, 0.0), (0.7071068,-0.7071068,0.0,0.0)]:
    q = a.ik([0.069, -0.010, 1.20], cand)
    print("IK hover", cand, None if q is None else np.round(q,3))
    if q is not None: print("   fk check tcp", np.round(a.tcp(q)[0],4), np.round(a.fk(q)[1],3))
