import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
HX, HY = 0.0717, 0.0
q = a.ik([HX, HY, 1.10], Q)
for i in range(3):
    code, err = a.move(q, 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3))
