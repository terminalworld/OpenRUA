import numpy as np, sys
from ctl import *
r=Robot("s2")
go="--go" in sys.argv
q0=topdown_quat(0.0)
gx,gy=-0.505,-0.1025
qside=R_quat(np.array([[1,0,0],[0,0,-1],[0,1,0]],float))
# backward chain from the grasp config
final=[0.29,0.33,-0.89,-2.67,-2.34,3.5,2.59]
zs=[1.081,1.10,1.15,1.30]
sols={}; seed=final
for z in zs:
    s=r.ik_solve([gx,gy,z],q0,seed,collide=True)
    if s is None: print("IK fail",z); sys.exit(1)
    print(z,np.round(s,2),"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max()); sols[z]=s; seed=s
back=r.ik_solve([-0.508,0.09,1.25],qside,r.arm_q(),collide=True); print("back",np.round(back,2))
print("transition delta",np.round(np.array(sols[1.30])-np.array(back),2))
plan=[("back",back)]+[(z,sols[z]) for z in [1.30,1.15,1.10,1.081]]
if go:
    for i,(lab,s) in enumerate(plan):
        r.move_joints(s,seconds=5 if i<2 else 4); p,q2=r.hand_pose(); print(lab,"hand",p.round(4),q2.round(3))
        if lab==1.15: r.snap("robot0_eye_in_hand","/workspace/s2_eih115.png"); r.snap("sideview","/workspace/s2_side115.png")
    r.snap("robot0_eye_in_hand","/workspace/s2_eih.png"); r.snap("sideview","/workspace/s2_side.png")
