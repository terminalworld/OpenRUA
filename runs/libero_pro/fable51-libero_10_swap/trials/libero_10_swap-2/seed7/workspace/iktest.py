import numpy as np
from lib import Robot
r = Robot("iktest")
p, q, _ = r.fk_world(); print("hand", np.round(p,4), np.round(q,4))
sol = r.ik_world(p, q); print("ik roundtrip:", np.round(sol,4) if sol else None, "current", np.round(r.arm_q(),4))
# test IK for a top-down pose above the knob
for quat in [(1,0,0,0), (0.7071,0.7071,0,0), (0.7071,-0.7071,0,0)]:
    sol = r.ik_world([-0.21, 0.194, 1.15], quat); print("knob-above", quat, np.round(sol,3) if sol else None)
    if sol: print("   fk check:", np.round(r.fk_world(sol)[0],4), np.round(r.fk_world(sol)[1],3))
