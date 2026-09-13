import numpy as np
from arm import Arm
a = Arm()
home = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
for i in range(4):
    code, err = a.move(home, 5.0)
    if err < 0.01: break
print("q", np.round(a.arm_q(),3), "tcp", np.round(a.tcp()[0],4))
