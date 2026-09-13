import numpy as np, sys
from rob import *
r = Robot("st")
q=r.arm_q(); print("q", np.round(q,3))
pos,R=r.fk_hand(); print("hand", np.round(pos,3)); print(np.round(R,2)); t,_=r.tcp(); print("tcp", np.round(t,3))
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
for c in sys.argv[1:]: r.snap(c)
