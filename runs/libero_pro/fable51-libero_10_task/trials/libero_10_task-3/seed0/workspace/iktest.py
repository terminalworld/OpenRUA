import numpy as np
from rob import Robot, rot_from_axes, R_to_quat
r=Robot("ikt")
q=r.arm_q(); pos,quat,R=r.fk_hand(q)
print("hand", np.round(pos,4), np.round(quat,4))
sol=r.ik_hand(pos,quat)
print("ik roundtrip:", None if sol is None else np.round(sol,4)); print("current:", np.round(q,4))
# test IK for a top-down grasp pose above the bottle: tcp at (-0.14,0.049,1.05), z down, fingers along world y
Rg=rot_from_axes([0,0,-1],[1,0,0])
print("Rg\n",np.round(Rg,3), R_to_quat(Rg))
for z in (1.20,1.10,1.00):
    s=r.ik_tcp([-0.14,0.049,z],Rg)
    print("tcp z",z,"->", None if s is None else np.round(s,3))
    if s is not None:
        p,_,_=r.tcp(s); print("   fk tcp check", np.round(p,4))
