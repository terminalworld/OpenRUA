import numpy as np
from rob import *
r = Robot("t3")
pos, q = r.hand_pose_world()
print("hand", np.round(pos,4), np.round(q,4), "tcp", np.round(r.tcp_world()[0],4))
print("IK(current hand) ->", np.round(r.ik_solve(pos, q), 4))
print("current          ->", np.round(r.joints(), 4))
for yaw in (0, math.pi/2):
    print("yaw", yaw, "quat", np.round(yaw_down_quat(yaw),4), "R", np.round(quat_R(*yaw_down_quat(yaw)),3).tolist())
