import numpy as np
from rob import *
from goto import ik_checked
r = Robot("dq")
R1 = np.load("snaps/R_side.npy")
q0 = r.arm_q()
q = ik_checked(r, [-0.201, 0.012, 1.04], R1, q0)
print("q0", np.round(q0,3)); print("q ", np.round(q,3)); print("diff", np.round(q-q0,3))
print("force", np.round(r.force(),2))
