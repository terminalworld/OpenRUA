import numpy as np, sys
from rob import Robot
r = Robot("look")
qn = np.load("snaps/quat_push.npy")
p = np.array([float(v) for v in sys.argv[1:4]])
q = r.ik(p, qn, seed=r.arm_q()); assert q is not None
r.move_q_corrected(q, seconds=3.0, iters=1)
print("TCP", r.tcp()[0].round(4))
