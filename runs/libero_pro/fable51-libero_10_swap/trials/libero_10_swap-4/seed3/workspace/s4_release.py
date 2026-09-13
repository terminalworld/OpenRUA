import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s4")
Q = topdown_quat(0.0)
tcp, _ = r.tcp_pose_world()
r.gripper(0.04)
print("wrench after release", np.round(r.read_wrench(), 3))
z_up = float(sys.argv[1]) if len(sys.argv) > 1 else 0.70
r.move_tcp([tcp[0], tcp[1], z_up], Q, seconds=3.0)
