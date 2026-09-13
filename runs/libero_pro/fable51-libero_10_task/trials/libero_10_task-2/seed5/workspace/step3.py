import sys, numpy as np
from rob import *
r = Robot()
delta = float(sys.argv[1])
gap = r.gripper(0.0)
q = r.arm_q()
q2 = list(q); q2[6] = q[6] + delta
print("j7", q[6], "->", q2[6])
r.move_q(q2, seconds=3.0)
print("gap after rotate", r.finger_gap())
r.gripper(0.04)
p, _ = r.tcp_pose(); print("tcp", np.round(p, 4))
