import numpy as np
from rob import Robot, rot_from_axes, R_to_quat, quat_to_R
def slerp(q0,q1,t):
    q0=np.asarray(q0,float); q1=np.asarray(q1,float)
    if q0@q1<0: q1=-q1
    d=np.clip(q0@q1,-1,1); th=np.arccos(d)
    if th<1e-6: return q0
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)
r=Robot("push2")
al=np.deg2rad(40)
z=np.array([0,np.cos(al),-np.sin(al)]); hy=np.array([1.0,0,0])
Rp=rot_from_axes(z, np.cross(hy,z))
p,_,R0=r.tcp(); print("tcp", np.round(p,4))
q=r.arm_q()
# 1 go high
q=r.move_tcp([p[0],p[1],1.35],R0,seed=q,max_jump=0.8)
if q is None: raise SystemExit("ik high")
# 2 rotate in steps while translating toward (0.0,-0.05,1.35)
q0=R_to_quat(R0); q1=R_to_quat(Rp)
start=np.array([p[0],p[1],1.35]); goal=np.array([0.0,-0.05,1.35])
for t in [0.25,0.5,0.75,1.0]:
    Rt=quat_to_R(slerp(q0,q1,t)); pt=start+(goal-start)*t
    q=r.move_tcp(pt,Rt,seed=q,max_jump=0.9)
    if q is None: raise SystemExit(f"ik rot t={t}")
# 3 approach
X=0.06; Z=0.955
for pt in [[X,-0.02,1.15],[X,0.02,1.02],[X,0.02,Z]]:
    q=r.move_tcp(pt,Rp,seed=q,max_jump=0.9)
    if q is None: raise SystemExit(f"ik approach {pt}")
print("wrench pre", np.round(r.wrench()[0],2))
for y in [0.07,0.11,0.15,0.19,0.23,0.245]:
    q=r.move_tcp([X,y,Z],Rp,seed=q,max_jump=0.6)
    if q is None: raise SystemExit(f"ik y={y}")
    print("wrench", np.round(r.wrench()[0],2), flush=True)
p,_,R=r.tcp()
q=r.move_tcp(p-0.10*R[:,2],R,seed=q,max_jump=0.6)
q=r.move_tcp([X,-0.05,1.20],R,seed=q,max_jump=0.9)
