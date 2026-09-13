import numpy as np, sys, math
from rob import *
r=Robot()
d=np.array([-0.823,-0.568,0.0]); d/=np.linalg.norm(d)
z=np.array([0,0,1.0])
u=(d*math.sin(math.radians(45))+z*math.cos(math.radians(45)))  # base->cork axis when tilted
a1=-u
a0=np.array([0,0,-1.0])
# rotation taking a0 -> a1
n=np.cross(a0,a1); s=np.linalg.norm(n); n/=s; th=math.atan2(s,a0@a1)
K=np.array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
Rt=np.eye(3)+math.sin(th)*K+(1-math.cos(th))*K@K
c1=Rt@d
print("u",u.round(3),"a1",a1.round(3),"c1",c1.round(3),"n",n.round(3),"th",math.degrees(th))
Rh=hand_R(a1,c1); qh=quat_from_R(Rh)
B=np.array([0.085,0.195,0.943])
H=B+0.235*u
print("H",H.round(4))
def go(p,q,secs):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    print("  sol",np.round(sol,3))
    code,err,n_=r.move_q_conv(sol,secs)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)} q={quat.round(3)} fingers={np.round(r.fingers(),4)}")
    return sol
if go(H+[0,0,0.06],qh,5.0) is None: sys.exit(1)
