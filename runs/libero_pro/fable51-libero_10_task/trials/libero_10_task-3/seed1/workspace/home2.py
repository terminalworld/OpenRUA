import numpy as np, rclpy, sys
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q_mid = q_home.copy(); q_mid[0] = q_now[0]
print("stage A (fold up, joint1 fixed):")
okA, _ = path_check(c, q_now, q_mid, n=12, z_floor=0.99)
print("stage B (swing joint1):")
okB, _ = path_check(c, q_mid, q_home, n=12, z_floor=0.99)
if okA and okB and "--go" in sys.argv:
    c.movej(q_mid, 5.0)
    c.movej(q_home, 6.0)
    print(fmt(*c.hand_pose()))
rclpy.shutdown()
