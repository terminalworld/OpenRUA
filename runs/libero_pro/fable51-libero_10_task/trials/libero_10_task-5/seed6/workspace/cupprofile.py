import numpy as np, sys
from scene import grab
import rclpy
node,cams=grab(["sideview","frontview"])
for c in cams:
    P=c.cloud().reshape(-1,3); P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.08)&(P[:,1]<0.12)&(P[:,2]>0.885)&(P[:,2]<1.06)
    Q=P[sel]
    print(c.name, len(Q))
    for lo in np.arange(0.885,1.05,0.01):
        s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)]
        if len(s)<3: continue
        print("  z %.3f-%.3f n=%4d x[%.3f %.3f] y[%.3f %.3f]"%(lo,lo+0.01,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
rclpy.shutdown()
