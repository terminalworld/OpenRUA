import numpy as np, sys
from scene import grab
import rclpy
node,cams=grab(["sideview","frontview","agentview","birdview"])
allP=[]
for c in cams:
    P=c.cloud().reshape(-1,3); P=P[np.isfinite(P).all(1)]
    sel=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.08)&(P[:,1]<0.12)&(P[:,2]>0.882)&(P[:,2]<1.0)
    allP.append(P[sel])
Q=np.concatenate(allP)
# estimate axis: rim points z>0.975, fit circle center by mean of extremes
rim=Q[Q[:,2]>0.98]; 
# exclude handle: y< -0.0
body=rim[rim[:,1]>0.0]
cx=(body[:,0].min()+body[:,0].max())/2; cy=(body[:,1].min()+body[:,1].max())/2
print("rim center est", cx, cy, "rim diam x", body[:,0].max()-body[:,0].min(), "y", body[:,1].max()-body[:,1].min())
for lo in np.arange(0.882,1.0,0.01):
    s=Q[(Q[:,2]>=lo)&(Q[:,2]<lo+0.01)&(Q[:,1]>cy-0.02)]  # exclude handle side
    if len(s)<5: continue
    r=np.hypot(s[:,0]-cx,s[:,1]-cy)
    print("z %.3f n=%4d r pct50/90/98/max: %.3f %.3f %.3f %.3f"%(lo,len(s),*np.percentile(r,[50,90,98]),r.max()))
np.save("cupQ.npy",Q)
rclpy.shutdown()
