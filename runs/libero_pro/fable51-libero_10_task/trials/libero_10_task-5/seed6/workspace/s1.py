import numpy as np, sys
from ctl import *
r=Robot("s1")
go="--go" in sys.argv
R=np.array([[1,0,0],[0,0,-1],[0,1,0]],float)
q=R_quat(R)
ax_y=-0.1025; ax_z=0.978; gx=-0.508
wps=[([gx,0.09,1.25],q),([gx,0.09,1.10],q),([gx,0.09,ax_z],q)]
seed=r.arm_q()
sols=[]
for pos,qq in wps:
    s=r.ik_solve(pos,qq,seed)
    if s is None: print("IK fail",pos); sys.exit(1)
    print(pos,"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max(),np.round(s,2)); seed=s; sols.append(s)
if go:
    for (pos,qq),s in zip(wps,sols):
        r.move_joints(s,seconds=4); p,q2=r.hand_pose(); print("  hand",p.round(4),q2.round(3))
    r.snap("robot0_eye_in_hand","/workspace/s1_eih.png"); r.snap("sideview","/workspace/s1_side.png")
