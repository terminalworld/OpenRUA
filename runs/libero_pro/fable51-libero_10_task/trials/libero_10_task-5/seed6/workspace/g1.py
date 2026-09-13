import numpy as np
from ctl import *
r=Robot("g1")
cx,cy=-0.1222,0.0429
xh=cx+0.047; yh=cy
q=topdown_quat(np.pi/2)
print("target quat",q.round(3))
ok=r.move_pose([xh,yh,1.16],q,seconds=4)
p,qq=r.hand_pose(); print("hand pose",p.round(4),qq.round(3)); print("arm q",np.round(r.arm_q(),3))
c,d,P,T=r.snap("robot0_eye_in_hand","/workspace/g1_eih.png")
c2,_,_,_=r.snap("frontview","/workspace/g1_front.png")
