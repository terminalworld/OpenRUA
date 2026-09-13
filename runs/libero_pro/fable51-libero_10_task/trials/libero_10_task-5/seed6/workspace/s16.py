import numpy as np, json
from ctl import *; from mp import *
from valid import check
r=Robot("s16"); pl=Planner(r)
print("attach:", attach_mug(pl))
print("valid now:", check(r, r.arm_q()))
target=np.array([-0.417,-0.097,1.33]); q=topdown_quat(0.0)
sols=[]
for i in range(8):
    seed=list(np.random.uniform(-1,1,7)*np.array([2,1,2,0.5,2,1,2])+np.array([0,0,0,-2,0,2,0]))
    s=r.ik_solve(target,q,seed,collide=True)
    if s is not None: sols.append(s); print("IK", np.round(s,2), "valid", check(r,s)[0])
json.dump(sols,open("carry_sols.json","w"))
