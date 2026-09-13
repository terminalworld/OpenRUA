import numpy as np
from rob import *
r = Robot("pick")
BOT = np.array([-0.166, 0.073])
yaw = np.arctan2(0.57, 0.82)
R = R_topdown(yaw)
r.gripper(0.04)
q0 = r.arm_q()
# hover above bottle
q1 = r.move_tcp([BOT[0], BOT[1], 1.22], R, 5.0, seed=q0)
if q1 is None: raise SystemExit("no IK hover")
print("q1", np.round(q1,3), "force", np.round(r.force(),2))
# descend to neck
q2 = r.move_tcp([BOT[0], BOT[1], 1.105], R, 3.0, seed=q1)
if q2 is None: raise SystemExit("no IK descend")
print("force", np.round(r.force(),2))
r.gripper(0.0)
print("gap after close", round(r.finger_gap(),4))
r.snap("agentview"); r.snap("sideview")
