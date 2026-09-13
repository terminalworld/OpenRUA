import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s8"); pl=Planner(r)
pl.scene([],remove=["mug","mug_body","mug_rim","mug_handle"]+[f"mugw{k}" for k in range(12)]+["mugw_bottom"])
print("scene",pl.scene(scene_objects(with_mug=False)+ring_objects(MUG_C[0],MUG_C[1],0.88,0.995,skip_angle=np.pi/2)+[mug_objects()[1]]))
lim=np.array(FJT["limits_rad"])
tip=np.array([MUG_C[0],MUG_C[1]+0.0485,0.972])
def pose(tipz,deg):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    t=tip.copy(); t[2]=tipz; return t-0.1034*zh, R_quat(np.stack([xh,yh,zh],1))
rng=np.random.default_rng(5)
res={}
for deg in [-30,-20,-10,0,10,20]:
    pos,q=pose(0.972,deg); best=[]
    for i in range(30):
        s=r.ik_solve(pos,q,rng.uniform(lim[:,0],lim[:,1]),collide=True)
        if s is None: continue
        s=np.array(s); best.append((np.minimum(s-lim[:,0],lim[:,1]-s).min(),s))
    best.sort(key=lambda t:-t[0]); res[deg]=best
    print("tilt",deg,"origin",pos.round(3),"nsol",len(best),[ (round(m,2),np.round(s,2).tolist()) for m,s in best[:2]])
