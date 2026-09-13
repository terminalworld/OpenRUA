import numpy as np, sys
from rob import *
r=Robot()
print("close gripper:", r.gripper(0.0))
R=hand_R((0,0,-1),(1,0,0)); q=quat_from_R(R)
print("target quat",q.round(4))
def go(p, secs=3.0, seed=None):
    sol=r.ik_hand(p,q,seed=seed)
    if sol is None: print("IK FAIL", p); sys.exit(1)
    code,err=r.move_q(sol,secs)
    pos,_,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} hand={pos.round(4)}")
    return sol
go([0.007,0.105,1.20],4.0)
go([0.007,0.105,1.058],3.0)
w0=r.wrench(); print("wrench before pull",w0.round(2))
for y in [0.08,0.055,0.03,0.005,-0.02]:
    go([0.007,y,1.058],2.0)
    print("  wrench",r.wrench().round(2))
go([0.007,-0.02,1.20],3.0)
print("fingers",r.fingers())
