import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('appr')
prev=np.array(r.arm_q())
from mugmodel import c,t,n_in,d
qh=hand_quat(d,-np.cross(t,d))
start=r.tcp()[0]; w0=r.wrench()[:3]
total=float(sys.argv[1])
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2),'dw',np.round(np.linalg.norm(w-w0),2)); return w
for k in np.arange(0.01,total+1e-6,0.01):
    w=go(start+k*d,1.2)
    if np.linalg.norm(w-w0)>float(sys.argv[2]): print('CONTACT at k',round(k,3)); break
