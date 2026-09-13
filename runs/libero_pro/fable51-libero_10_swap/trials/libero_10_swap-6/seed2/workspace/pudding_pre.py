import numpy as np, sys, rclpy
sys.path.insert(0,"/workspace")
from arm import Arm, down_quat
a = Arm()
HOME = np.array([0,-0.161,0,-2.445,0,2.227,0.785])
PITCH = 20
p, _ = a.tcp_pose()
print("1. rise"); a.line([p[0], p[1], 0.80], down_quat(0, 0), T=3)
print("2. pre-grasp (home-seeded)")
sol = a.solve_ik([-0.222, 0.022, 0.62], down_quat(0, PITCH), seed=HOME)
print("   sol", sol.round(3)); a.move_joints(sol, 5)
p, q = a.tcp_pose(); print("TCP", p.round(4), q.round(3))
