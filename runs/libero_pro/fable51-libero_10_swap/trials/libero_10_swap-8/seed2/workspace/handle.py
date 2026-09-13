import numpy as np
for cam in ["birdview","agentview","sideview"]:
    P=np.load(f"{cam}_cloud.npy").reshape(-1,3); P=P[np.isfinite(P).all(1)]
    print("==",cam)
    for name,(x0,x1,y0,y1,yb) in {"A":(-0.26,-0.13,-0.32,-0.243,-0.243),"B":(-0.10,0.0,0.14,0.215,0.215)}.items():
        m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.905)
        Q=P[m]
        print(f" handle {name}: n={len(Q)}")
        if len(Q)==0: continue
        for z0 in np.arange(0.94,1.05,0.01):
            s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
            if len(s)<2: continue
            print(f"   z {z0:.2f}: n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
