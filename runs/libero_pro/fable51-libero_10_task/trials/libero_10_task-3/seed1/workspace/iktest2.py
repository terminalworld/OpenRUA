import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q()); print("q0 ", q0.round(3))
pos, R = c.hand_pose()
for i in range(4):
    q = c.solve_ik(pos, R, seed=q0, tries=1)
    print("sol", np.array(q).round(3), " fk:", c.hand_pose(q)[0].round(4))
rclpy.shutdown()
