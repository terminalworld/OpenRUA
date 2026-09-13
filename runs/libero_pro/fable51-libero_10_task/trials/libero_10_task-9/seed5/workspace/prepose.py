import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('pre')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.324,0.947]); n_in=np.array([-0.34,0.94,0.0]); yhat=np.array([0.94,0.34,0.0])
b=np.deg2rad(45); d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0]); qh=hand_quat(d,-np.cross(yhat,d))
tcp=p+0.018*n_in
k=float(sys.argv[1]) if len(sys.argv)>1 else 0.06
r.gripper(0.08)
s=best_ik(r,tcp-k*d,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
r.move_joints(s,4.0); print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
