import numpy as np, rclpy
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
print("now:", q_now.round(3))
ok, _ = path_check(c, q_now, q_home, n=20, z_floor=0.98)
rclpy.shutdown()
