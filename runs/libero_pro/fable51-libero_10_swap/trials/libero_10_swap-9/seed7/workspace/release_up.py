import numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("ru")
r.gripper(0.04)
s = r.ik((-0.05, 0.13, 1.25), Q(45), seed=r.joints(), at_tcp=True)
r.move_joints([s], [4.0])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
