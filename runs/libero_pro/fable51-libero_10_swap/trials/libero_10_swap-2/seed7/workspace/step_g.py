import numpy as np, sys
from lib import Robot, quat_R
r = Robot("stepg")
for quat in [(0.5556, 0.8315, 0, 0), (0.8315, 0.5556, 0, 0)]:
    quat = np.array(quat)/np.linalg.norm(quat)
    sol = r.ik_world([-0.0296, -0.242, 1.2], quat)
    if sol is None: print(quat, "no IK"); continue
    p, q, _ = r.fk_world(sol); R = quat_R(q)
    print("quat", np.round(quat,4), "sol", np.round(sol,3), "finger axis (hand y) in world:", np.round(R[:,1],3), "hand z:", np.round(R[:,2],3))
