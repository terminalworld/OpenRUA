import numpy as np, rclpy
from ctl import *
c = Ctl()
q0 = np.array(c.arm_q())
pos, R = c.hand_pose()
print("cur  :", fmt(pos, R))
q = c.solve_ik(pos, R, seed=q0, tries=1)
p2, R2 = c.hand_pose(q)
print("sol  :", fmt(p2, R2), " rot err deg:", np.degrees((R.inv()*R2).magnitude()).round(2))
rclpy.shutdown()
