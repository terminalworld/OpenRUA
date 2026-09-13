import numpy as np, sys
from rob import *
r=Robot()
d=np.array([-0.823,-0.568,0.0])
Rg=hand_R((0,0,-1),d); qg=quat_from_R(Rg)
sol=r.ik_hand([-0.165,0.061,1.30],qg)
code,err,n=r.move_q_conv(sol,3.0)
pos,quat,_=r.fk_hand(); print(f"lift code={code} jerr={err:.4f} retries={n} hand={pos.round(4)} fingers={r.fingers()}")
