import numpy as np
from rob import *
r = Robot("lift")
q0 = r.arm_q(); tcp, R = r.tcp()
print("tcp", np.round(tcp,3))
q = r.move_tcp([tcp[0], tcp[1], 1.32], R, 4.0, seed=q0)
print("gap", round(r.finger_gap(),4), "force", np.round(r.force(),2))
img, P = r.cloud("sideview")
Z=P[...,2]; X=P[...,0]; Y=P[...,1]
m=(Z>0.905)&(Z<1.1)&(X>-0.25)&(X<-0.1)&(Y>0.0)&(Y<0.15)
print("bottle-region points left on table (should be ~0):", m.sum())
m2=(Z>1.05)&(Z<1.35)&(X>-0.25)&(X<-0.05)&(Y>-0.05)&(Y<0.2)
print("points 1.05-1.35 (bottle hanging):", m2.sum(), "zmin", Z[m2].min() if m2.any() else None)
