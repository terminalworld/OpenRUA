import numpy as np
from arm import *
a = Arm("away2")
q0 = a.arm_q()
for t in ([-0.25, -0.30, 1.30], [-0.30, -0.25, 1.30], [-0.20, -0.20, 1.25]):
    q = a.ik(t, R_DOWN_FX, seed=q0)
    if q is not None:
        print("target", t, "jump", np.abs(q-q0).max().round(3)); break
code, err = a.move_q(q, seconds=6.0)
if code != 0 or err > 0.02: a.move_q(q, seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4), "fingers", a.finger_gap())
