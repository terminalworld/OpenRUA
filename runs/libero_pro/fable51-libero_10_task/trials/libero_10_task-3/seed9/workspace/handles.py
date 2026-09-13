import numpy as np
from rob import *
r = Robot("hd")
pts=[]
for cam in ["agentview","frontview","sideview","birdview"]:
    img,P=r.cloud(cam); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.12)&(P[:,0]<0.12)&(P[:,1]>0.15)&(P[:,1]<0.222)&(P[:,2]>0.93)&(P[:,2]<1.13)
    pts.append(P[m]); print(cam, m.sum())
P=np.concatenate(pts)
for z in np.arange(0.93,1.13,0.01):
    s=P[(P[:,2]>=z)&(P[:,2]<z+0.01)]
    if len(s)<5: print(f"z={z:.2f} n={len(s)}"); continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
print("handles (y<0.215):")
H=P[P[:,1]<0.215]
for z in np.arange(0.99,1.12,0.01):
    s=H[(H[:,2]>=z)&(H[:,2]<z+0.01)]
    if len(s)<5: continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
