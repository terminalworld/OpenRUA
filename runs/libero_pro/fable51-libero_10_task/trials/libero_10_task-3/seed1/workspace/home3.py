import numpy as np, rclpy, sys
from ctl import *
c = Ctl()
q_now = np.array(c.arm_q())
# stage A0: lift the hand straight up 15 cm in current orientation (IK near current config)
pos, R = c.hand_pose()
target = pos + np.array([0, 0, 0.15])
q_up = np.array(c.solve_ik(target, R, seed=q_now))
print("q_now", q_now.round(3)); print("q_up ", q_up.round(3), "diff", np.abs(q_up-q_now).max().round(3))
okA0, _ = path_check(c, q_now, q_up, n=10, z_floor=0.99, verbose=False)
q_home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
q_mid = q_home.copy(); q_mid[0] = q_now[0]
okA, _ = path_check(c, q_up, q_mid, n=12, z_floor=0.99, verbose=False)
okB, _ = path_check(c, q_mid, q_home, n=12, z_floor=0.99, verbose=False)
print(okA0, okA, okB)
if okA0 and okA and okB and "--go" in sys.argv:
    c.movej(q_up, 4.0)
    c.movej(q_mid, 5.0)
    c.movej(q_home, 6.0)
    print(fmt(*c.hand_pose()))
rclpy.shutdown()
