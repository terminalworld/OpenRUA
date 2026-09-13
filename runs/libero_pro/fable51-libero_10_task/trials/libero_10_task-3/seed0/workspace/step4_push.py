import numpy as np
from rob import Robot, rot_from_axes
r=Robot("push")
r.gripper(0.0)
al=np.deg2rad(40)
z=np.array([0,np.cos(al),-np.sin(al)]); hy=np.array([1.0,0,0])
Rp=rot_from_axes(z, np.cross(hy,z)); print("Rp\n",np.round(Rp,3))
X=0.06; Z=0.955
q=r.move_tcp([X,-0.02,1.15],Rp)
if q is None: raise SystemExit("ik1")
q=r.move_tcp([X,0.02,Z],Rp,seed=q)
if q is None: raise SystemExit("ik2")
print("wrench pre", np.round(r.wrench()[0],2))
for y in [0.07,0.11,0.15,0.19,0.23,0.245]:
    q=r.move_tcp([X,y,Z],Rp,seed=q)
    if q is None: raise SystemExit(f"ik y={y}")
    print("wrench", np.round(r.wrench()[0],2), flush=True)
# retreat
p,_,R=r.tcp()
q=r.move_tcp(p-0.10*R[:,2],R,seed=q)
q=r.move_tcp([X,-0.05,1.20],R,seed=q)
