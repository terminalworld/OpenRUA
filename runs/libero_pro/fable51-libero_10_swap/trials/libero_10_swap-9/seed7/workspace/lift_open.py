import numpy as np
from ctl import Robot, log
r=Robot("lo"); r.gripper(0.04)
hx,hq=r.hand_pose(); s=r.ik(hx+[0,0,0.15],hq); r.move_joints([s],[4.0]); log("hand",np.round(r.hand_pose()[0],3))
