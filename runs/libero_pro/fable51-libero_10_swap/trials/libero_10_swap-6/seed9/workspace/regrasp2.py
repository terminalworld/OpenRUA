import numpy as np
from arm import Arm, R_quat
from stage8 import G, R1
arm = Arm("regrasp2")
arm.open()
p, _ = arm.tcp_world()
G2 = np.array([G[0], G[1], 0.548])
up = np.array([p[0], p[1], 0.548])
arm.move_path([(up, R_quat(R1))], secs=3)
p, _ = arm.tcp_world()
arm.move_path([(p + (G2 - p) * i / 2, R_quat(R1)) for i in range(1, 3)], secs=4)
p, q = arm.tcp_world(); print("at grasp: tcp", p.round(4), "quat", np.round(q, 3), flush=True)
arm.close(); print("fingers", arm.finger_gap(), flush=True)
