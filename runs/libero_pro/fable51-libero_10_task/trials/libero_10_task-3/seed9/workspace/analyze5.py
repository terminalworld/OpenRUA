import numpy as np
for name in ["agentview","sideview","birdview"]:
    P=np.load(f"snaps/{name}_xyz.npy").reshape(-1,3)
    m=(P[:,2]>0.93)&(P[:,2]<1.135)&(P[:,0]>-0.21)&(P[:,0]<-0.12)&(P[:,1]>0.02)&(P[:,1]<0.12)
    Q=P[m]
    print(name, "bottle pts", len(Q))
    for zlo in np.arange(0.93,1.14,0.02):
        s=Q[(Q[:,2]>=zlo)&(Q[:,2]<zlo+0.02)]
        if len(s): print(f"  z {zlo:.2f}-{zlo+0.02:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
