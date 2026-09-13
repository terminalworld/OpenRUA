import numpy as np, sys
P=np.load(f"{sys.argv[1]}_cloud.npy").reshape(-1,3)
P=P[np.isfinite(P).all(1)]
# pot A region and pot B region boxes (xy), profile width per z-bin
boxes={"potA":(-0.30,-0.10,-0.32,-0.10),"potB":(-0.15,0.05,0.12,0.35),"stove":(0.09,0.33,-0.08,0.15)}
for name,(x0,x1,y0,y1) in boxes.items():
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.903)
    Q=P[m]
    print(name, "n=",len(Q), "ztop=%.3f"%Q[:,2].max() if len(Q) else "")
    for z0 in np.arange(0.90,1.08,0.01):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(s)<3: continue
        print(f"  z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
