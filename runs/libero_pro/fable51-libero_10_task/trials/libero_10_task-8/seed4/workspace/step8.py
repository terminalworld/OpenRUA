import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
TX, TY = -0.055, -0.254
q0 = a.arm_q()
qa = a.ik([0.01, -0.13, 1.10], Q, seed=q0)
qb = a.ik([TX, TY, 1.10], Q, seed=qa)
for i in range(3):
    code, err = a.move([qa, qb], 5.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers())
