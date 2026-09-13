import numpy as np
from arm import Arm
a = Arm()
Q = (0.7071068, 0.7071068, 0.0, 0.0)
q1 = a.ik([-0.055, -0.254, 1.30], Q)
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
for i in range(3):
    code, err = a.move([q1, home], 6.0)
    if err < 0.01: break
print("tcp", np.round(a.tcp()[0],4))
