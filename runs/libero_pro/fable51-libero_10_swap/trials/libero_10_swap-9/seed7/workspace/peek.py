import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
r = Robot("pk")
x,y,z,th = map(float, sys.argv[1:5])
s = r.ik((x,y,z), quat_from_axes(a_of(th),[1,0,0]), seed=r.joints(), at_tcp=True); assert s is not None
r.move_joints([s],[5.0]); log("hand", np.round(r.hand_pose()[0],3))
