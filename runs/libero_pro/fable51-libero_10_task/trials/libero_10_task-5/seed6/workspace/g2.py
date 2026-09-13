import numpy as np
from ctl import *
r=Robot("g2")
xh,yh=-0.0740,0.0448
q=topdown_quat(np.pi/2)
for z in (1.11,1.07):
    r.move_pose([xh,yh,z],q,seconds=3)
p,qq=r.hand_pose(); print("hand",p.round(4),qq.round(3)); print("fingers",r.finger())
r.snap("robot0_eye_in_hand","/workspace/g2_eih.png"); r.snap("frontview","/workspace/g2_front.png")
