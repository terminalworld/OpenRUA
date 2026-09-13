import numpy as np
from rob import Robot, topdown_quat, quat_to_R
r = Robot("dbg")
Q = topdown_quat(0.0)
q = r.ik_world([-0.13, -0.231, 0.65], Q, at_tcp=True)
print("sol", np.round(q, 4))
pos, fq, _ = r.hand_pose_world(q)
print("FK hand pos", np.round(pos, 4), "quat", np.round(fq, 4), "requested", Q)
print(np.round(quat_to_R(*fq), 3))
tcp, _ = r.tcp_pose_world(q); print("tcp", np.round(tcp, 4))
