import numpy as np, sys
from ctl import *
r=Robot("iks")
lim=np.array(FJT["limits_rad"])
pos=[float(v) for v in sys.argv[1:4]]; yaw=float(sys.argv[4]) if len(sys.argv)>4 else 0.0
q=topdown_quat(yaw)
rng=np.random.default_rng(0); best=[]
for i in range(40):
    seed=rng.uniform(lim[:,0],lim[:,1]) if i else r.arm_q()
    s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: continue
    s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min()
    best.append((marg,s))
best.sort(key=lambda t:-t[0])
print(len(best),"solutions")
for m,s in best[:8]: print("margin %.2f"%m,np.round(s,2))
