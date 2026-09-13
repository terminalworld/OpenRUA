import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('nudge')
prev=np.array(r.arm_q())
t=np.array([0.866,0.5,0.0]); n_in=np.array([-0.5,0.866,0.0])
b=np.deg2rad(45); d=np.cos(b)*n_in-np.sin(b)*np.array([0,0,1.0]); qh=hand_quat(d,-np.cross(t,d))
xh=-np.cross(t,d); yh=np.cross(d,xh)
print('xhat',np.round(xh,3),'yhat',np.round(yh,3))
dd,dx,dy=map(float,sys.argv[1:4])
cur=r.tcp()[0]
tgt=cur+dd*d+dx*xh+dy*yh
s=best_ik(r,tgt,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,2.5); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
