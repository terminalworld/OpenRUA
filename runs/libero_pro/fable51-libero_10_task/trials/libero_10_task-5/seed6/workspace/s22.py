import numpy as np, sys
from ctl import *; from wr import Wrench
exec(open("s20.py").read().split("RX,RY,RZTOP")[0].replace('r=Robot("s20")','r=Robot("s22")'))
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]; print("tip now", tip.round(4))
for z in [1.085,1.080,1.076]:
    go([tip[0],tip[1],z]); 
    if w.get()[2]>-2.0: print("pressing hard"); break
print("pre-release", w.get(), np.round(r.finger(),4)); r.gripper(0.08)
print("post-release", w.get())
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]
for z in [1.11,1.16,1.22]: go([tip[0],tip[1],z])
q=r.arm_q()
for i in range(3): r.move_joints(q,seconds=2,retries=0)
print("settled", w.get())
