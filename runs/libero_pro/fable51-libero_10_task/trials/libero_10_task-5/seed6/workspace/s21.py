import numpy as np, sys
from ctl import *; from wr import Wrench
exec(open("s20.py").read().split("RX,RY,RZTOP")[0].replace('r=Robot("s20")','r=Robot("s21")'))
f=r.gripper(0.0)
if abs(f[0])>0.012 or abs(f[0])<0.002: print("grasp looks wrong", f); sys.exit()
p,_=r.hand_pose(); tip=p+[0,0,-0.1034]
# lift to base above wall tops, shift -x, lower until contact
for z in [1.10,1.13,1.16]: go([tip[0],tip[1],z])
X=tip[0]-0.007
for z in [1.16,1.12,1.09,1.08,1.075,1.07]:
    if not go([X,tip[1],z]): break
    if w.get()[2]>-4.7: print("contact"); break
