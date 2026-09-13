import numpy as np, time
from rob import *
r = Robot("stage1")
print("fingers before:", r.fingers())
r.gripper(0.04)
POT_A = np.array([-0.196, -0.200])
Q_FX = (0.7071, -0.7071, 0, 0)   # fingers along world x, hand z down
q0 = r.arm_q()
pre = r.ik_tcp_world((POT_A[0], POT_A[1], 1.15), Q_FX, seed=q0, timeout=5)
print("pre-grasp q:", None if pre is None else np.round(pre, 3))
if pre is None: raise SystemExit("no IK")
r.move_q(pre, 4.0)
tcp, quat = r.tcp_world(); print("tcp now", np.round(tcp, 4), np.round(quat, 3))
r.snap("robot0_eye_in_hand"); r.depth("robot0_eye_in_hand")
r.snap("agentview", "/workspace/agentview_s1.png")
