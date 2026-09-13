import numpy as np, json
from ctl import *; from mp import *
from valid import check
r=Robot("s13"); pl=Planner(r)
print("scene rm:", pl.scene([], remove=[f"mugw{i}" for i in range(12)]+["mugw_bottom","mug_handle","mug_body","mug_rim"]))
tip=np.array([-0.548,0.1045,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
seed=r.arm_q(); print("cur", np.round(seed,2))
chain=[(0.972,0,list(map(float,seed)))]
for z in [1.00,1.03,1.06,1.09,1.12,1.16,1.20,1.25]:
    best=None
    for deg in [0,3,6,10,-3,-6]:
        pos,q=pose(z,deg); s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        d=np.abs(np.array(s)-np.array(seed)).max()
        if d<0.6 and (best is None or d<best[0]): best=(d,deg,s)
    if best is None: print("no IK at",z); break
    d,deg,s=best; print("z %.3f tilt %3d delta %.2f %s"%(z,deg,d,np.round(s,2)))
    chain.append((z,deg,list(map(float,s)))); seed=s
json.dump(chain,open("chain3.json","w"))
