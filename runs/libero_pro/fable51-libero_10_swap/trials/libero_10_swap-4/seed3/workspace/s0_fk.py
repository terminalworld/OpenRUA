import numpy as np
from rob import Robot, topdown_quat, quat_to_R
r = Robot("s0")
print("joints", np.round(r.joints(), 4))
pos, q, frame = r.hand_pose_world()
print("hand world", np.round(pos, 4), "quat", np.round(q, 4), "frame", frame)
print("hand R:\n", np.round(quat_to_R(*q), 3))
print("tcp world", np.round(r.tcp_pose_world()[0], 4))
print("topdown yaw0", np.round(topdown_quat(0), 4), "R:\n", np.round(quat_to_R(*topdown_quat(0)), 3))
print("fingers", r.fingers(), "wrench", r.read_wrench())
