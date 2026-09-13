import numpy as np, sys, json, subprocess
sys.path.insert(0,'.')
from rob import *
r=Robot('pinch2')
prev=np.array(r.arm_q())
from mugmodel import c,R,t,n_in,p,d



qh=hand_quat(d,-np.cross(t,d))
tcp=p+0.018*n_in
mode=sys.argv[1]
def go(pt,secs=2.5):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2)); return w
if mode=='pre':
    r.gripper(0.08); go(tcp-0.06*d,4.0)
elif mode=='approach':
    w0=r.wrench()[:3]
    for k in [0.045,0.03,0.02,0.01,0.0]:
        w=go(tcp-k*d,1.5)
        if np.linalg.norm(w-w0)>2.0: print('CONTACT at k',k); break
elif mode=='close':
    r.gripper(0.0); print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
elif mode=='lift':
    cur=r.tcp()[0]
    go(cur+np.array([0,0,0.04]),2.0); go(cur+np.array([0,0,0.12]),2.5)
    print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
if mode=='shift':
    dy=float(sys.argv[2])           # metres along hand-y (image right)
    yh=-t
    cur=r.tcp()[0]; go(cur+dy*yh,2.0)
