import numpy as np, sys
from rob import *
r = Robot("resend")
q = np.array([float(x) for x in sys.argv[1].split(",")]); sec = float(sys.argv[2])
print("now ", np.round(r.arm_q(),3)); print("goal", np.round(q,3))
r.move_q(q, sec)
print("after", np.round(r.arm_q(),3))
pos, Rh = r.fk_hand(); print("hand", np.round(pos,3)); print(np.round(Rh,2))
