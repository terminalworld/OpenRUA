import numpy as np, sys
from lib import Robot
delta = float(sys.argv[1])  # radians to ADD to joint7
r = Robot("stepd")
q = r.arm_q(); print("q7 before:", round(q[6],4), "gap:", round(r.finger_gap(),4))
q[6] += delta
r.move_q(q, sec=max(1.5, abs(delta)*2))
print("q7 after:", round(r.arm_q()[6],4), "gap:", round(r.finger_gap(),4), "wrench:", np.round(r.wrench()[0],3), np.round(r.wrench()[1],3))
