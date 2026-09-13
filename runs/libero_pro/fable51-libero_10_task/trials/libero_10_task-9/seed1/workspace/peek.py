import numpy as np
from rob import Robot
r = Robot("peek")
qn = np.load("snaps/quat_ins.npy")
q = r.ik(np.array([-0.06, 0.08, 1.03]), qn, seed=r.arm_q())
assert q is not None
r.move_q_corrected(q, seconds=4.0, iters=2)
print("TCP", r.tcp()[0].round(4))
