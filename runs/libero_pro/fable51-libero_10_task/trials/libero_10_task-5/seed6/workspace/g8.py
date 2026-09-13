import numpy as np, sys
from ctl import *
r=Robot("g8")
q0=topdown_quat(0.0)
r.move_pose([-0.30,0.15,1.35],q0,seconds=4)
for cam in ["birdview","sideview","robot0_robotview","frontview"]:
    c,d,P,T=r.snap(cam,f"/workspace/g8_{cam}.png"); np.save(f"g8_{cam}_P.npy",P)
P=np.concatenate([np.load(f"g8_{c}_P.npy").reshape(-1,3) for c in ["birdview","sideview"]]); P=P[np.isfinite(P).all(1)]
sel=P[(P[:,0]>-0.70)&(P[:,0]<-0.47)&(P[:,1]>-0.30)&(P[:,1]<0.05)&(P[:,2]>0.89)&(P[:,2]<1.1)]
print("mug candidate pts",len(sel),"x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(sel[:,0].min(),sel[:,0].max(),sel[:,1].min(),sel[:,1].max(),sel[:,2].min(),sel[:,2].max()))
xs=np.arange(-0.68,-0.48,0.01); ys=np.arange(-0.22,0.02,0.01)
print("      "+" ".join("%3d"%round(y*100) for y in ys))
for x in xs:
    row=[]
    for y in ys:
        s=P[(P[:,0]>=x)&(P[:,0]<x+0.01)&(P[:,1]>=y)&(P[:,1]<y+0.01)]
        row.append("%3d"%round((s[:,2].max()-0.88)*100) if len(s) else "  .")
    print("%5.2f "%x+" ".join(row))
