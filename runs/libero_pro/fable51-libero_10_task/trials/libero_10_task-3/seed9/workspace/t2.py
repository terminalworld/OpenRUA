import numpy as np
from rob import *
r = Robot("t2")
q0 = r.arm_q()
pos, R = r.fk_hand(); print("hand world", np.round(pos,4))
q = r.ik_hand(pos, R, seed=q0)
print("ik roundtrip q", None if q is None else np.round(q,3), "vs", np.round(q0,3))
# try IK for a top-down pose above the bottle
target = np.array([-0.166, 0.073, 1.25])
for yaw in [0, np.pi/4, -np.pi/4, np.pi/2]:
    q = r.ik_hand(target, R_topdown(yaw), seed=q0)
    print("yaw", yaw, "->", None if q is None else np.round(q,3))
