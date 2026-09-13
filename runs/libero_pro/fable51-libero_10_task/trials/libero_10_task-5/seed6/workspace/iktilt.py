import numpy as np, sys
from ctl import *
r=Robot("ikt")
lim=np.array(FJT["limits_rad"])
tipx=float(sys.argv[1]) if len(sys.argv)>1 else -0.508
tip=np.array([tipx,-0.1025,0.978])
def pose(th):
    zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    R=np.stack([xh,yh,zh],1); return tip-0.1034*zh, R_quat(R)
rng=np.random.default_rng(1)
for deg in [-20,-10,0,10,20,30]:
    pos,q=pose(np.radians(deg)); best=[]
    for i in range(40):
        seed=rng.uniform(lim[:,0],lim[:,1])
        s=r.ik_solve(pos,q,seed,collide=True)
        if s is None: continue
        s=np.array(s); marg=np.minimum(s-lim[:,0],lim[:,1]-s).min(); best.append((marg,s))
    best.sort(key=lambda t:-t[0])
    print("tilt",deg,"origin",pos.round(3),"nsol",len(best))
    for m,s in best[:3]: print("   margin %.2f"%m,np.round(s,2))
