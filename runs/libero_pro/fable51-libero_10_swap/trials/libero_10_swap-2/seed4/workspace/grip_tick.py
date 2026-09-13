import sys
from moka_plan import *
m = Mover("grip_tick")
w = float(sys.argv[1]); secs = float(sys.argv[2]) if len(sys.argv)>2 else 2.0
print("fingers before", np.round(m.fingers(),4))
m.gripper(w)
q = m.arm_q(); m.goto_q(q, secs)   # hold pose to advance the sim clock
print("fingers after", np.round(m.fingers(),4), "wrench", np.round(wrench(m),2))
