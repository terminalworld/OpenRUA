import numpy as np
from rob import Robot, topdown_quat
r = Robot("s5b")
Q = topdown_quat(0.0)
pinch = [-0.229, 0.070]
q = r.ik_world([pinch[0], pinch[1], 0.70], Q)
print("IK sol", np.round(q, 4), "\ncurrent", np.round(r.joints(), 4))
for attempt in range(3):
    code, err = r.move_joints(q, 5.0)
    print("attempt", attempt, "code", code, "err", err, "joints", np.round(r.joints(), 4))
    if err < 0.02: break
tcp, _ = r.tcp_pose_world(); print("tcp", np.round(tcp, 4))
