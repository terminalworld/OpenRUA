import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
q = a.ik([0.069, -0.010, 1.20], Q)
print("target q", np.round(q,3), "current", np.round(a.arm_q(),3))
for i in range(3):
    code, err = a.move(q, 6.0)
    if err < 0.01: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3))
