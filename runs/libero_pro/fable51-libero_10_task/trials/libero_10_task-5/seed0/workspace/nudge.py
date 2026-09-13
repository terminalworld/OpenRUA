import sys, numpy as np
from kin import Robot
r = Robot("nudge")
d = np.array(list(map(float, sys.argv[1:4])))
p, qt, _ = r.tcp(); w0 = r.wrench()
s = r.ik(p + d, qt, seed=r.arm_q()); assert s
r.move(s, 1.5, retries=1); r.report("nudged")
print("fingers", np.round(r.fingers(), 4), "dwrench", (r.wrench() - w0).round(2))
