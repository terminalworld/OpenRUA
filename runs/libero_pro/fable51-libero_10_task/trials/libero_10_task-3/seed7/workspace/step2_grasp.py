import numpy as np, sys
from rob import *
r=Robot()
print("open gripper:", r.gripper(0.04))
bottle=np.array([-0.165,0.061])
d=np.array([-0.823,-0.568,0.0])
Rg=hand_R((0,0,-1),d); qg=quat_from_R(Rg)
print("grasp R\n",Rg.round(3))
def go(p, q, secs=3.0):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL", p); sys.exit(1)
    code,err=r.move_q(sol,secs)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} hand={pos.round(4)} q={quat.round(3)}")
    return sol
go([bottle[0],bottle[1],1.30],qg,4.0)
go([bottle[0],bottle[1],1.14],qg,3.0)
print("close:", r.gripper(0.0))
r.spin(0.5); print("fingers", r.fingers())
