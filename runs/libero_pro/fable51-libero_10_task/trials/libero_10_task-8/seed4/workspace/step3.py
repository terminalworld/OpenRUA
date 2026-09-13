import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
CX, CY = 0.0719, 0.0582
q = a.ik([CX, CY, 1.12], Q)
for i in range(3):
    code, err = a.move(q, 4.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), np.round(a.fk()[1],3), "fingers", a.fingers())
