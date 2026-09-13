import sys, numpy as np
from arm import Arm, down_quat
a = Arm("iktest3")
x,y,z = map(float, sys.argv[1:4])
q0 = a.arm_q(); print("cur q", q0.round(3))
for n,s in {"cur": q0, "home": np.array([0,-0.161,0,-2.445,0,2.227,0.785]), "ready": np.array([0,-0.785,0,-2.356,0,1.571,0.785])}.items():
    q = a.ik([x,y,z], down_quat(0), seed=s)
    print(n, None if q is None else q.round(3), flush=True)
