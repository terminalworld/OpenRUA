import numpy as np
from rob import *
r = Robot("vi")
pts=[]
for cam in ["agentview","frontview","birdview","sideview"]:
    img,P=r.cloud(cam); P=P.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.12)&(P[:,0]<0.12)&(P[:,1]>0.07)&(P[:,1]<0.26)&(P[:,2]>0.928)&(P[:,2]<1.05)
    pts.append(P[m]); print(cam, m.sum())
P=np.concatenate(pts)
# exclude the drawer walls (x beyond +-0.095, y<0.098) and the hand (z>1.0 & ... ) : keep interior points
I=P[(np.abs(P[:,0])<0.095)&(P[:,1]>0.098)&(P[:,1]<0.245)]
print("interior pts", len(I), "z max", I[:,2].max().round(3))
for z in np.arange(0.93,1.0,0.01):
    s=I[(I[:,2]>=z)&(I[:,2]<z+0.01)]
    if len(s)<5: continue
    print(f"z={z:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# anything above 0.98 within the full box footprint (rim height) besides walls?
A=P[(P[:,2]>0.985)]
print("pts above 0.985:", len(A))
if len(A): print(" x", A[:,0].min().round(3), A[:,0].max().round(3), " y", A[:,1].min().round(3), A[:,1].max().round(3), " z", A[:,2].max().round(3))
