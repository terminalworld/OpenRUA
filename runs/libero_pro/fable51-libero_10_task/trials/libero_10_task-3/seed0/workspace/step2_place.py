import numpy as np
from rob import Robot, rot_from_axes
r=Robot("place")
print("fingers", np.round(r.finger(),4))
p,quat,R0=r.tcp(); print("tcp start", np.round(p,4))
A=np.array([-0.107,0.095]); C=np.array([0.099,0.254])
dh=(C-A)/np.linalg.norm(C-A); print("diag dir", np.round(dh,3), "len", np.linalg.norm(C-A))
al=np.deg2rad(25)
d=np.array([np.cos(al)*dh[0], np.cos(al)*dh[1], -np.sin(al)])
hy=np.array([-dh[1], dh[0], 0.0])
Rp=rot_from_axes(d, np.cross(hy,d))
print("Rp\n", np.round(Rp,3))
bottom=np.array([0.062,0.228,0.949]); L=0.20
tcp_final=bottom-L*d; print("tcp_final", np.round(tcp_final,4))
# 1 lift
q=r.move_tcp([-0.137,0.049,1.30],R0)
# 2 rotate high
q=r.move_tcp([-0.10,0.06,1.30],Rp,seed=q)
if q is None: raise SystemExit("rotate IK failed")
# 3 above
q=r.move_tcp(tcp_final+[0,0,0.07],Rp,seed=q)
if q is None: raise SystemExit("above IK failed")
print("wrench", np.round(r.wrench()[0],2))
# 4 descend
q=r.move_tcp(tcp_final,Rp,seed=q)
if q is None: raise SystemExit("final IK failed")
print("wrench at bottom", np.round(r.wrench()[0],2), "fingers", np.round(r.finger(),4))
