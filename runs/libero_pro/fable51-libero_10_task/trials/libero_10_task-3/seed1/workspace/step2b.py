import numpy as np, rclpy
from ctl import *
c = Ctl()
th = np.radians(17)
R = R_from_axes(zaxis=[0, np.cos(th), -np.sin(th)], yaxis=[0, -np.sin(th), -np.cos(th)])
q0 = np.array(c.arm_q()); print("now:", q0.round(3))
q = np.array(c.solve_ik([-0.08, -0.12, 1.08], R))
print("sol:", q.round(3)); print("diff:", (q-q0).round(3))
code, err = c.movej(q, 4.0)
print("after:", np.array(c.arm_q()).round(3))
print(fmt(*c.hand_pose()))
rclpy.shutdown()
