import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('ikc')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.314,0.947]); n_in=np.array([-0.31,0.95,0.0]); yhat=np.array([0.95,0.31,0.0])
for beta in [45]:
    b=np.deg2rad(beta); d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0]); qh=hand_quat(d,-np.cross(yhat,d)); tcp=p+0.018*n_in
    for k in [0.08,0.05,0.0]:
        s=best_ik(r,tcp-k*d,qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
        print(beta,k,None if s is None else np.round(s,2))
