import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('rel'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.22-p[0],0,p[0]+0.16); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh2,seed=prev,prefer=prev,avoid=lambda L: not door_only(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
cur=r.tcp()[0]
go((cur[0],0.276,cur[2]),1.5)
go((cur[0],0.276,1.030),2.0)
r.gripper(0.08); print('OPEN fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
go((cur[0],0.22,1.030),2.0)
go((cur[0],0.12,1.08),2.5)
