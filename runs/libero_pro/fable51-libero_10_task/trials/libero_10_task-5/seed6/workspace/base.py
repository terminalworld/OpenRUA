import numpy as np
from ctl import *
r=Robot("base")
cx,cy=-0.1222,0.0429
for cam in ["frontview","sideview","agentview"]:
    _,_,P,T=r.snap(cam)
    print(cam, "cam at", T[:3,3].round(3))
    Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
    near=np.hypot(Q[:,0]-cx,Q[:,1]-cy)<0.08
    for lo in np.arange(0.882,0.99,0.008):
        s=Q[near&(Q[:,2]>=lo)&(Q[:,2]<lo+0.008)]
        if len(s)<5: continue
        # radius on +y side, +x side, -x side
        ry=s[:,1].max()-cy; rxp=s[:,0].max()-cx; rxm=cx-s[:,0].min()
        print("  z %.3f n=%3d  r(+y)=%.4f r(+x)=%.4f r(-x)=%.4f"%(lo,len(s),ry,rxp,rxm))
