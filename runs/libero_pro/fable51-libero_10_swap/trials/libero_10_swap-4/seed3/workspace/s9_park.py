import numpy as np, sys
from rob import Robot, topdown_quat
r = Robot("s9")
Q = topdown_quat(0.0)
x, y, z = map(float, sys.argv[1:4])
for attempt in range(3):
    q = r.ik_world([x, y, z], Q)
    if q is None: raise SystemExit("IK failed")
    code, err = r.move_joints(q, 4.0)
    if err < 0.02: break
print("tcp", np.round(r.tcp_pose_world()[0], 4))
