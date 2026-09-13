import numpy as np
from arm import *
a = Arm("place2")
qs = np.load("place_qs.npy")
code, err = a.move_q(qs[1], seconds=6.0, via=[qs[0]])
if code != 0 or err > 0.02:
    code, err = a.move_q(qs[1], seconds=3.0)
p, R = a.tcp(); print("tcp", np.round(p,4)); print("fingers", a.finger_gap())
