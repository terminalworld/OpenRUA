import numpy as np, rob, sys
from scipy.spatial.transform import Rotation as Rot, Slerp
r=rob.Robot(); p,q=r.fk(); q0=r.arm_q()
print("cur q",np.round(q0,2))
try: print("ik of current:",np.round(r.ik(list(p),list(q),seed=q0,timeout=3,avoid=False),2))
except Exception as e: print("ik current failed",e)
lo=np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi=np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9]); rng=np.random.default_rng(3)
# target: hand pointing +y, xh=-Z, at several positions
Mt=np.stack([[0,0,-1.0],[-1.0,0,0],[0,1.0,0]],1); qt=list(Rot.from_matrix(Mt).as_quat())
for pos in ([-0.135,-0.523,1.09],[-0.135,-0.50,1.10],[-0.10,-0.50,1.10],[-0.135,-0.48,1.15],[-0.135,-0.45,1.20]):
    sols=[]
    for k in range(15):
        try: qq=r.ik(pos,qt,seed=list(rng.uniform(lo,hi)),timeout=0.5,avoid=False)
        except Exception: qq=None
        if qq is not None: sols.append(np.round(qq,2))
    print(pos,len(sols),sols[:2])
