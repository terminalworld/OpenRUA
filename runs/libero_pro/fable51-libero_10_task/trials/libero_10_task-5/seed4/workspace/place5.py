import numpy as np
from arm import *
a = Arm("place5")
q0 = a.arm_q(); p0, R0 = a.tcp(q0)
t = p0 + np.array([0, -0.10, 0.0])
q = a.ik(t, R0, seed=q0); print("jump", np.abs(q-q0).max().round(3))
code, err = a.move_q(q, seconds=4.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4))
