import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry3'); prev=np.array(r.arm_q())
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
stage=sys.argv[1]
X=-0.073
if stage=='shift':
    go((X,0.20,1.0226),2.0)
elif stage=='insert':
    w0=r.wrench()[:3]
    for y in [0.23,0.26,0.28,0.30]:
        w=go((X,y,1.0226),2.0)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
elif stage=='release':
    r.gripper(0.08); print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
    cur=r.tcp()[0]
    go((cur[0],0.22,cur[2]),2.5); go((cur[0],0.12,cur[2]+0.05),2.5)
elif stage=='retry':
    cur=r.tcp()[0]
    go((X,0.22,cur[2]),2.0); go((X,0.22,1.038),2.0)
    w0=r.wrench()[:3]
    for y in [0.25,0.27,0.29,0.31]:
        w=go((X,y,1.038),2.0)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
elif stage=='probe':
    X2=float(sys.argv[2]); Z2=float(sys.argv[3])
    cur=r.tcp()[0]
    go((cur[0],0.265,cur[2]),1.5); go((X2,0.265,Z2),1.5)
    w0=r.wrench()[:3]
    for y in [0.285,0.30,0.315]:
        w=go((X2,y,Z2),1.5)
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
