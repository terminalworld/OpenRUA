import sys, numpy as np
from ctl import Robot, quat_down, log
r = Robot("lk")
x,y,z = map(float, sys.argv[1:4])
s = r.ik((x,y,z), quat_down(90), seed=r.joints(), at_tcp=True); assert s is not None
r.move_joints([s],[5.0]); log("hand", np.round(r.hand_pose()[0],3))
