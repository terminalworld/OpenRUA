import numpy as np
from rob import Robot, topdown_quat
r = Robot("neutral")
qn = topdown_quat([1, 0, 0])
q = r.ik(np.array([-0.15, -0.05, 1.40]), qn, seed=r.arm_q())
assert q is not None, "IK fail"
r.move_q_corrected(q, seconds=5.0, iters=1)
print("TCP", r.tcp()[0].round(3), "q", np.round(r.arm_q(), 2))
