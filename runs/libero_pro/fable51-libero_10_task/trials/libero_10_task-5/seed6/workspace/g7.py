import numpy as np, sys
from ctl import *
r=Robot("g7")
q0=topdown_quat(0.0)
f=r.gripper(0.04)
p,qq=r.hand_pose(); print("hand",p.round(4))
r.move_pose([p[0],p[1],1.36],q0,seconds=3)
for cam in ["agentview","sideview","frontview","birdview"]:
    c,d,P,T=r.snap(cam,f"/workspace/g7_{cam}.png"); np.save(f"g7_{cam}_P.npy",P)
