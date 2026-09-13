import sys, numpy as np
from kin import Robot
QD = np.array([0.7071, 0.7071, 0.0, 0.0])
r = Robot("park")
p = r.report("start")
s = r.ik((p[0], p[1], 1.30), QD, seed=r.arm_q()); assert s; r.move(s, 3.0, retries=3)
s = r.ik((-0.20, 0.35, 1.30), QD, seed=s); assert s; r.move(s, 5.0, retries=3)
r.report("parked")
