import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('carry2'); prev=np.array(r.arm_q())
d2=np.array([-0.5,0.7071,-0.5]); x2=np.array([-0.5,-0.7071,-0.5]); d2/=np.linalg.norm(d2); x2/=np.linalg.norm(x2); qh2=hand_quat(d2,x2)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        dx=max(-0.20-p[0],0,p[0]+0.14); dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if np.sqrt(dx*dx+dy*dy+dz*dz)<0.05: return False
    return True
def go(pt,secs,avoid):
    global prev
    s=best_ik(r,np.array(pt),qh2,seed=prev,prefer=prev,avoid=avoid)
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
stage=sys.argv[1]
if stage=='A':
    for w in [(-0.10,-0.20,1.15),(-0.043,0.05,1.15),(-0.043,0.10,1.023),(-0.043,0.20,1.023)]:
        go(w,4.0,lambda L: not clear_of_door(L))
elif stage=='B':
    w0=r.wrench()[:3]
    for y in [0.23,0.26,0.28,0.30]:
        w=go((-0.043,y,1.023),2.0,lambda L: not door_only(L))
        print('dw',np.round(np.linalg.norm(w-w0),2))
        if np.linalg.norm(w-w0)>2.5: print('CONTACT'); break
