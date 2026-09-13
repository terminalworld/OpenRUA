import numpy as np
from rob import *
r = Robot("pf")
pts=[]
for cam in ["agentview","frontview","sideview","galleryview"]:
    try:
        img, P = r.cloud(cam)
    except Exception as e:
        print(cam, "fail", e); continue
    P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.23)&(P[:,0]<-0.10)&(P[:,1]>0.0)&(P[:,1]<0.15)&(P[:,2]>1.0)&(P[:,2]<1.30)
    print(cam, m.sum()); pts.append(P[m])
P=np.concatenate(pts)
np.save("snaps/bottle_air.npy",P)
G=np.array([-0.166,0.073])
for z in np.arange(1.02,1.28,0.01):
    s=P[(P[:,2]>=z)&(P[:,2]<z+0.01)]
    if len(s)==0: print(f"z={z:.2f} none"); continue
    rr=np.hypot(s[:,0]-G[0],s[:,1]-G[1])
    print(f"z={z:.2f} n={len(s):4d} cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f} r_med={np.median(rr):.3f} r90={np.percentile(rr,90):.3f}")
