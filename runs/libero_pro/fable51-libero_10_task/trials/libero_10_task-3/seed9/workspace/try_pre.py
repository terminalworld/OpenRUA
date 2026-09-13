import numpy as np
from rob import *
from goto import goto
r = Robot("tp")
R1 = np.load("snaps/R_side.npy")
goto(r, [-0.201, 0.012, 1.04], R1, seconds=4, resend=2)
print("q", np.round(r.arm_q(),3), "force", np.round(r.force(),2))
